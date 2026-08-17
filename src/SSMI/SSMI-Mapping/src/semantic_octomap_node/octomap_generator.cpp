#include <semantic_octomap_node/octomap_generator.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy
#include <unordered_map>
#include <vector>

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::OctomapGenerator()
    : octomap_(0.05), max_range_(1.), raycast_range_(1.),
      raycast_clearing_enabled_(true),
      dynamic_free_updates_(64), dynamic_free_confirmations_(1) {}

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::~OctomapGenerator(){}

template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::setUseSemanticColor(bool use)
{
    octomap_.setUseSemanticColor(use);
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::isUseSemanticColor()
{
    return octomap_.isUseSemanticColor();
}

template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::setWriteSemantics(bool write)
{
    octomap_.setWriteSemantics(write);
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::doesWriteSemantics()
{
    return octomap_.doesWriteSemantics();
}

template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
    const octomap::ColorOcTreeNode::Color dynamic_color(255, 0, 255);

    // Transform first, then select one original point per OctoMap key. The
    // semantic_color field contains packed RGB bits inside a float; applying
    // arithmetic (as PCL VoxelGrid does) corrupts those bits, often to black.
    // Categorical fields must only ever be copied, never averaged.
    CLOUD input_cloud;
    pcl::fromPCLPointCloud2(*cloud, input_cloud);
    CLOUD transformed_cloud;
    pcl::transformPointCloud(input_cloud, transformed_cloud, sensorToWorld);
    CLOUD pcl_cloud;
    pcl_cloud.reserve(transformed_cloud.size());
    std::vector<float> selected_distance_sq;
    selected_distance_sq.reserve(transformed_cloud.size());
    std::unordered_map<uint64_t, size_t> key_to_index;
    key_to_index.reserve(transformed_cloud.size());

    const auto semanticBits = [](const typename CLOUD::PointType& point) {
        uint32_t bits;
        std::memcpy(&bits, &point.semantic_color, sizeof(uint32_t));
        return bits;
    };
    const auto isDynamic = [&dynamic_color, &semanticBits](const typename CLOUD::PointType& point) {
        const uint32_t bits = semanticBits(point);
        return ((bits >> 16) & 0xff) == dynamic_color.r &&
               ((bits >> 8) & 0xff) == dynamic_color.g &&
               (bits & 0xff) == dynamic_color.b;
    };
    const auto semanticPriority = [this, &isDynamic, &semanticBits](
                                      const typename CLOUD::PointType& point) {
        if (obstacle_semantic_colors_.count(semanticBits(point) & 0x00ffffffu) != 0)
            return 2;
        return isDynamic(point) ? 1 : 0;
    };
    const auto packKey = [](const octomap::OcTreeKey& key) {
        return (static_cast<uint64_t>(key[0]) << 32) |
               (static_cast<uint64_t>(key[1]) << 16) |
               static_cast<uint64_t>(key[2]);
    };

    const float origin_x = static_cast<float>(sensorToWorld(0,3));
    const float origin_y = static_cast<float>(sensorToWorld(1,3));
    const float origin_z = static_cast<float>(sensorToWorld(2,3));
    for (typename CLOUD::const_iterator it = transformed_cloud.begin();
         it != transformed_cloud.end(); ++it)
    {
        if (!std::isfinite(it->x) || !std::isfinite(it->y) || !std::isfinite(it->z))
            continue;

        octomap::OcTreeKey key;
        if (!octomap_.coordToKeyChecked(it->x, it->y, it->z, key))
            continue;
        const uint64_t packed_key = packKey(key);
        const float dx = it->x - origin_x;
        const float dy = it->y - origin_y;
        const float dz = it->z - origin_z;
        const float distance_sq = dx*dx + dy*dy + dz*dz;

        const auto existing = key_to_index.find(packed_key);
        if (existing == key_to_index.end())
        {
            key_to_index.emplace(packed_key, pcl_cloud.size());
            pcl_cloud.push_back(*it);
            selected_distance_sq.push_back(distance_sq);
            continue;
        }

        const size_t index = existing->second;
        const int incoming_priority = semanticPriority(*it);
        const int selected_priority = semanticPriority(pcl_cloud[index]);
        // Confirmed static obstacles outrank dynamic occluders, and dynamic
        // detections outrank ordinary semantics. Within one group retain the
        // nearest sample so categorical packed RGB fields are never averaged.
        if ((incoming_priority > selected_priority) ||
            (incoming_priority == selected_priority &&
             distance_sq < selected_distance_sq[index]))
        {
            pcl_cloud[index] = *it;
            selected_distance_sq[index] = distance_sq;
        }
    }

    //tf::Vector3 originTf = sensorToWorldTf.getOrigin();
    //octomap::point3d origin(originTf[0], originTf[1], originTf[2]);
    octomap::point3d origin(origin_x, origin_y, origin_z);
    octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
    std::unordered_map<uint64_t, octomap::OcTreeKey> current_dynamic_keys;
    int endpoint_count = 0; // total number of endpoints inserted
    for(typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
    {
        // Check if the point is invalid
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            float dist = sqrt((it->x - origin.x())*(it->x - origin.x()) + (it->y - origin.y())*(it->y - origin.y()) + (it->z - origin.z())*(it->z - origin.z()));
            // Check if the point is in max_range
            if(dist <= max_range_)
            {
                octomap::ColorOcTreeNode::Color color_obs(it->r, it->g, it->b);
                octomap::ColorOcTreeNode::Color class_obs;
                // Get semantics
                uint32_t rgb;
                std::memcpy(&rgb, &it->semantic_color, sizeof(uint32_t));
                class_obs.r = (rgb >> 16) & 0x0000ff;
                class_obs.g = (rgb >> 8)  & 0x0000ff;
                class_obs.b = (rgb)       & 0x0000ff;
            
                octomap::OcTreeKey occupied_key;
                const bool has_occupied_key = octomap_.coordToKeyChecked(
                    it->x, it->y, it->z, occupied_key);
                SemanticsOcTreeNode* existing_node = has_occupied_key
                    ? octomap_.search(occupied_key) : NULL;
                bool dynamic_occludes_static = false;
                if (class_obs == dynamic_color && existing_node != NULL &&
                    existing_node->isSemanticsSet())
                {
                    const octomap::ColorOcTreeNode::Color existing_semantic =
                        existing_node->getSemantics().data[0].color;
                    const uint32_t existing_bits =
                        (static_cast<uint32_t>(existing_semantic.r) << 16) |
                        (static_cast<uint32_t>(existing_semantic.g) << 8) |
                        static_cast<uint32_t>(existing_semantic.b);
                    dynamic_occludes_static =
                        obstacle_semantic_colors_.count(existing_bits) != 0;
                }

                // A dynamic return at an already-confirmed wall is occlusion,
                // not evidence that the wall disappeared. Keep the old static
                // endpoint and let a later explicit free/terrain policy decide
                // whether it may ever be cleared or reclassified.
                SemanticsOcTreeNode* node = dynamic_occludes_static
                    ? existing_node
                    : octomap_.updateNode(
                        it->x, it->y, it->z, true, class_obs, color_obs, false);

                if (!dynamic_occludes_static && node != NULL && class_obs == dynamic_color)
                {
                    // A current dynamic detection is authoritative for this
                    // occupied voxel. Replace (do not average) both colors and
                    // make label 6 / magenta the top semantic immediately.
                    const float occupancy_log_odds = node->getLogOdds();
                    octomap::SemanticsLogOdds dynamic_semantics;
                    dynamic_semantics.data[0] = octomap::ColorWithLogOdds(
                        dynamic_color, occupancy_log_odds);
                    node->setSemantics(dynamic_semantics);
                    node->setColor(dynamic_color);
                    node->setLogOdds(occupancy_log_odds);

                    if (has_occupied_key)
                    {
                        current_dynamic_keys.emplace(packKey(occupied_key), occupied_key);
                        dynamic_free_confirmation_counts_[packKey(occupied_key)] = 0;
                    }
                }
                else if (has_occupied_key)
                {
                    // A current non-dynamic endpoint at the same key is an
                    // occupied static observation, not a ghost to clear.
                    previous_dynamic_keys_.erase(packKey(occupied_key));
                    dynamic_free_confirmation_counts_.erase(packKey(occupied_key));
                }
            
                endpoint_count++;
            }
            
            if (raycast_clearing_enabled_)
                raycast_cloud.push_back(it->x, it->y, it->z);
        }
    }

    // A local grid has already been fused upstream and does not encode a
    // sensor line of sight for every cell. Its points are occupied endpoints
    // only; absence from the moving local window is not free-space evidence.
    if (!raycast_clearing_enabled_)
    {
        previous_dynamic_keys_.clear();
        dynamic_free_confirmation_counts_.clear();
        return;
    }

    // Remember old dynamic nodes before the normal free-space update. A
    // decrease afterwards proves that the current scan actually traversed
    // the voxel; occluded old positions are deliberately left untouched.
    struct StaleDynamicNode
    {
        octomap::OcTreeKey key;
        float log_odds;
    };
    std::unordered_map<uint64_t, StaleDynamicNode> stale_dynamic_nodes;
    for (const auto& previous : previous_dynamic_keys_)
    {
        if (current_dynamic_keys.find(previous.first) != current_dynamic_keys.end())
            continue;
        SemanticsOcTreeNode* node = octomap_.search(previous.second);
        if (node != NULL)
            stale_dynamic_nodes.emplace(
                previous.first, StaleDynamicNode{previous.second, node->getLogOdds()});
    }

    // Do ray casting for points in raycast_range_
    if(raycast_cloud.size() > 0)
        octomap_.insertPointCloud(raycast_cloud, origin, raycast_range_, true);

    std::vector<uint64_t> cleared_dynamic_keys;
    for (const auto& stale : stale_dynamic_nodes)
    {
        SemanticsOcTreeNode* node = octomap_.search(stale.second.key);
        if (node == NULL)
        {
            cleared_dynamic_keys.push_back(stale.first);
            continue;
        }
        if (node->getLogOdds() >= stale.second.log_odds - 1e-6f)
        {
            dynamic_free_confirmation_counts_[stale.first] = 0;
            continue;
        }

        const int confirmations = ++dynamic_free_confirmation_counts_[stale.first];
        if (confirmations < dynamic_free_confirmations_)
            continue;

        // The current scan has already supplied a genuine free-space hit.
        // First retain the normal probabilistic updates, then force the old
        // dynamic endpoint to an explicit free state. This prevents highly
        // confident semantic nodes from merely fading while remaining
        // occupied in RViz.
        for (int i = 0;
             i < dynamic_free_updates_ && node != NULL && octomap_.isNodeOccupied(*node);
             ++i)
            node = octomap_.updateNode(stale.second.key, false);

        if (node != NULL)
        {
            node->setSemantics(octomap::SemanticsLogOdds());
            node->setLogOdds(octomap_.getClampingThresMinLog());
            node->setColor(0, 0, 0);
            cleared_dynamic_keys.push_back(stale.first);
        }
    }
    for (uint64_t key : cleared_dynamic_keys)
    {
        previous_dynamic_keys_.erase(key);
        dynamic_free_confirmation_counts_.erase(key);
    }
    for (const auto& current : current_dynamic_keys)
        previous_dynamic_keys_[current.first] = current.second;

    // Directly resetting a leaf must be reflected in its ancestors before
    // serialization, otherwise a stale occupied parent can remain visible.
    if (!cleared_dynamic_keys.empty())
        octomap_.updateInnerOccupancy();
    
    /* updates inner node occupancy and colors
    if(endpoint_count > 0)
        octomap_.updateInnerOccupancy();*/
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::get_ray_RLE(const octomap::point3d& origin, const octomap::point3d& end, semantic_octomap::RayRLE& rayRLE_msg)
{
    if (octomap_.get_ray_RLE(origin, end, rayRLE_msg))
    {
        return true;
    } else {
        return false;
    }
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::save(const char* filename) const
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open()){
        std::cout << "Writing octomap to " << filename << std::endl;
        octomap_.write(outfile);
        outfile.close();
        std::cout << "Color tree written " << filename << std::endl;
        return true;
    }
    else {
        std::cout << "Could not open " << filename  << " for writing" << std::endl;
        return false;
    }
}

//Explicit template instantiation
template class OctomapGenerator<PCLSemantics, SemanticOctree>;
