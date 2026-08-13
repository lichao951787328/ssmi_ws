#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <semantic_octomap/GetRLE.h>
#include <semantic_octomap/GetTraversabilityRLE.h>
#include <semantic_traversability_octree/SemanticTraversabilityOcTree.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using Rgb = std::array<uint8_t, 3>;

const std::array<Rgb, 19> kCityscapesPalette = {{
    {{128, 64, 128}}, {{244, 35, 232}}, {{70, 70, 70}},
    {{102, 102, 156}}, {{190, 153, 153}}, {{153, 153, 153}},
    {{250, 170, 30}}, {{220, 220, 0}}, {{107, 142, 35}},
    {{152, 251, 152}}, {{70, 130, 180}}, {{220, 20, 60}},
    {{255, 0, 0}}, {{0, 0, 142}}, {{0, 0, 70}},
    {{0, 60, 100}}, {{0, 80, 100}}, {{0, 0, 230}},
    {{119, 11, 32}}
}};

octomap::ColorOcTreeNode::Color toColor(const Rgb& value)
{
    return octomap::ColorOcTreeNode::Color(value[0], value[1], value[2]);
}

uint64_t packKey(const octomap::OcTreeKey& key)
{
    return (static_cast<uint64_t>(key[0]) << 32) |
           (static_cast<uint64_t>(key[1]) << 16) |
           static_cast<uint64_t>(key[2]);
}

bool hasField(const sensor_msgs::PointCloud2& cloud,
              const std::string& name, uint8_t datatype)
{
    for (const sensor_msgs::PointField& field : cloud.fields)
    {
        if (field.name == name)
            return field.datatype == datatype;
    }
    return false;
}

uint32_t labelFromSemanticColor(const octomap::ColorOcTreeNode::Color& color)
{
    for (uint32_t label = 0; label < kCityscapesPalette.size(); ++label)
    {
        if (color == toColor(kCityscapesPalette[label]))
            return label;
    }
    return std::numeric_limits<uint32_t>::max();
}

struct ScanVoxel
{
    octomap::OcTreeKey key;
    std::unordered_map<uint32_t, uint32_t> semantic_votes;
    float max_traversability = 0.0f;
    float traversability_sum = 0.0f;
    uint32_t traversability_samples = 0;
};
}  // namespace

class SemanticTraversabilityOctomapNode
{
public:
    SemanticTraversabilityOctomapNode()
        : nh_(), private_nh_("~"), tf_listener_(),
          cloud_subscriber_(nullptr), tf_filter_(nullptr)
    {
        loadParameters();
        if (!loadInitialTree())
            resetTree();

        full_map_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
        color_map_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_color", 1, true);
        traversability_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("traversability_voxels", 1, true);
        semantic_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("semantic_voxels", 1, true);
        combined_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "semantic_traversability_voxel_cloud", 1, true);
        occupancy_map_pub_ =
            nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_map_2D", 1, true);
        traversability_map_pub_ =
            nh_.advertise<nav_msgs::OccupancyGrid>("traversability_map_2D", 1, true);
        traversability_mean_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
            "traversability_mean_map_2D", 1, true);

        legacy_rle_service_ = nh_.advertiseService(
            "querry_RLE", &SemanticTraversabilityOctomapNode::legacyRleCallback, this);
        traversability_rle_service_ = nh_.advertiseService(
            "query_traversability_rle",
            &SemanticTraversabilityOctomapNode::traversabilityRleCallback, this);
        reset_service_ = nh_.advertiseService(
            "reset_semantic_traversability_octomap",
            &SemanticTraversabilityOctomapNode::resetCallback, this);

        cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
            nh_, input_topic_, 3);
        tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(
            *cloud_subscriber_, tf_listener_, world_frame_id_, 5);
        tf_filter_->registerCallback(
            boost::bind(&SemanticTraversabilityOctomapNode::cloudCallback, this, _1));

        ROS_INFO_STREAM("Semantic traversability OctoMap: " << input_topic_
                        << " -> " << world_frame_id_ << ", resolution="
                        << resolution_ << " m, ray clearing disabled for local grid input");
    }

    ~SemanticTraversabilityOctomapNode()
    {
        delete tf_filter_;
        delete cloud_subscriber_;
    }

    bool save() const
    {
        if (save_path_.empty())
            return true;
        std::ofstream stream(save_path_, std::ios::binary);
        if (!stream.is_open())
        {
            ROS_ERROR_STREAM("Unable to open traversability map for writing: " << save_path_);
            return false;
        }
        tree_->setWriteExtendedData(true);
        const bool success = tree_->write(stream);
        ROS_INFO_STREAM("Saved SemanticTraversabilityOcTree to " << save_path_);
        return success;
    }

private:
    void loadParameters()
    {
        private_nh_.param<std::string>("input_topic", input_topic_, "/grids_points");
        private_nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
        private_nh_.param<std::string>("save_path", save_path_,
                                       "/tmp/semantic_traversability_map.ot");
        private_nh_.param<std::string>("load_path", load_path_, "");
        private_nh_.param("resolution", resolution_, 0.2);
        private_nh_.param("max_range", max_range_, 15.0);
        private_nh_.param("clamping_thres_min", clamping_thres_min_, 0.0001);
        private_nh_.param("clamping_thres_max", clamping_thres_max_, 0.99);
        private_nh_.param("occupancy_thres", occupancy_thres_, 0.5);
        private_nh_.param("prob_hit", prob_hit_, 0.7);
        private_nh_.param("prob_miss", prob_miss_, 0.4);
        private_nh_.param("semantic_phi", semantic_phi_, -0.1);
        private_nh_.param("semantic_psi", semantic_psi_, 1.0);
        private_nh_.param("traversability_rise_alpha", rise_alpha_, 0.70);
        private_nh_.param("traversability_fall_alpha", fall_alpha_, 0.15);
        private_nh_.param("traversability_merge_epsilon", merge_epsilon_, 0.05);
        private_nh_.param("confidence_merge_epsilon", confidence_merge_epsilon_, 0.10);
        private_nh_.param("observation_confidence", observation_confidence_, 1.0);
        private_nh_.param("enable_pruning", enable_pruning_, true);
        private_nh_.param("publish_voxel_clouds", publish_voxel_clouds_, true);
        private_nh_.param("publish_traversability_2d", publish_traversability_2d_, true);
        private_nh_.param("min_ground_z", min_ground_z_, -1.0);
        private_nh_.param("max_ground_z", max_ground_z_, 1.0);
        private_nh_.param("occupancy_min_z", occupancy_min_z_, 1.0);
        private_nh_.param("occupancy_max_z", occupancy_max_z_, 3.5);

        int invalid_label = -1;
        private_nh_.param("invalid_semantic_label", invalid_label, -1);
        invalid_semantic_label_ = static_cast<uint32_t>(invalid_label);
    }

    void resetTree()
    {
        tree_.reset(new octomap::SemanticTraversabilityOcTree(resolution_));
        configureTree();
    }

    void configureTree()
    {
        tree_->setClampingThresMin(clamping_thres_min_);
        tree_->setClampingThresMax(clamping_thres_max_);
        tree_->setOccupancyThres(occupancy_thres_);
        tree_->setProbHit(prob_hit_);
        tree_->setProbMiss(prob_miss_);
        tree_->setSemanticMinProbability(clamping_thres_min_);
        tree_->setSemanticMaxProbability(clamping_thres_max_);
        tree_->setPhi(semantic_phi_);
        tree_->setPsi(semantic_psi_);
        tree_->setTraversabilityRiseAlpha(rise_alpha_);
        tree_->setTraversabilityFallAlpha(fall_alpha_);
        tree_->setTraversabilityMergeEpsilon(merge_epsilon_);
        tree_->setConfidenceMergeEpsilon(confidence_merge_epsilon_);
    }

    bool loadInitialTree()
    {
        if (load_path_.empty())
            return false;
        std::unique_ptr<octomap::AbstractOcTree> loaded(
            octomap::AbstractOcTree::read(load_path_));
        octomap::SemanticTraversabilityOcTree* typed =
            dynamic_cast<octomap::SemanticTraversabilityOcTree*>(loaded.get());
        if (typed == nullptr)
        {
            ROS_ERROR_STREAM("Map " << load_path_
                             << " is not a SemanticTraversabilityOcTree; starting empty");
            return false;
        }
        loaded.release();
        tree_.reset(typed);
        resolution_ = tree_->getResolution();
        configureTree();
        ROS_INFO_STREAM("Loaded SemanticTraversabilityOcTree from " << load_path_
                        << " with " << tree_->size() << " nodes");
        return true;
    }

    bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        resetTree();
        ROS_INFO("Semantic traversability OctoMap reset");
        return true;
    }

    bool legacyRleCallback(semantic_octomap::GetRLE::Request& request,
                           semantic_octomap::GetRLE::Response& response)
    {
        const octomap::point3d origin(
            request.origin.x, request.origin.y, request.origin.z);
        for (const geometry_msgs::Point& endpoint : request.endPoints)
        {
            semantic_octomap::RayRLE ray;
            if (tree_->getLegacyRayRLE(
                    origin, octomap::point3d(endpoint.x, endpoint.y, endpoint.z), ray))
                response.RLE_list.push_back(ray);
        }
        return true;
    }

    bool traversabilityRleCallback(
        semantic_octomap::GetTraversabilityRLE::Request& request,
        semantic_octomap::GetTraversabilityRLE::Response& response)
    {
        const octomap::point3d origin(
            request.origin.x, request.origin.y, request.origin.z);
        for (const geometry_msgs::Point& endpoint : request.end_points)
        {
            semantic_octomap::TraversabilityRayRLE ray;
            if (tree_->getTraversabilityRayRLE(
                    origin, octomap::point3d(endpoint.x, endpoint.y, endpoint.z), ray))
                response.rays.push_back(ray);
        }
        return true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        if (!validateFields(*cloud))
            return;

        tf::StampedTransform sensor_to_world_tf;
        try
        {
            tf_listener_.lookupTransform(world_frame_id_, cloud->header.frame_id,
                                         cloud->header.stamp, sensor_to_world_tf);
        }
        catch (const tf::TransformException& exception)
        {
            ROS_ERROR_STREAM("Transform error for /grids_points: " << exception.what());
            return;
        }

        Eigen::Matrix4f sensor_to_world;
        pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);
        const Eigen::Vector3f origin(sensor_to_world(0, 3), sensor_to_world(1, 3),
                                     sensor_to_world(2, 3));

        std::unordered_map<uint64_t, ScanVoxel> voxels;
        const size_t point_count = static_cast<size_t>(cloud->width) * cloud->height;
        voxels.reserve(point_count);

        sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> traversability_iterator(
            *cloud, "traversability");
        sensor_msgs::PointCloud2ConstIterator<uint32_t> semantic_iterator(
            *cloud, "semantic_lable");

        size_t rejected_points = 0;
        for (size_t index = 0; index < point_count;
             ++index, ++x_iterator, ++y_iterator, ++z_iterator,
             ++traversability_iterator, ++semantic_iterator)
        {
            if (!std::isfinite(*x_iterator) || !std::isfinite(*y_iterator) ||
                !std::isfinite(*z_iterator))
            {
                ++rejected_points;
                continue;
            }
            const Eigen::Vector4f local(*x_iterator, *y_iterator, *z_iterator, 1.0f);
            const Eigen::Vector4f world = sensor_to_world * local;
            if (!std::isfinite(world.x()) || !std::isfinite(world.y()) ||
                !std::isfinite(world.z()) ||
                (max_range_ > 0.0 &&
                 (world.head<3>() - origin).norm() > max_range_))
            {
                ++rejected_points;
                continue;
            }

            octomap::OcTreeKey key;
            if (!tree_->coordToKeyChecked(world.x(), world.y(), world.z(), key))
            {
                ++rejected_points;
                continue;
            }
            ScanVoxel& voxel = voxels[packKey(key)];
            voxel.key = key;

            const uint32_t label = *semantic_iterator;
            if (label != invalid_semantic_label_ && label < kCityscapesPalette.size())
                ++voxel.semantic_votes[label];

            if (std::isfinite(*traversability_iterator))
            {
                const float cost = std::max(0.0f, std::min(1.0f, *traversability_iterator));
                voxel.max_traversability =
                    std::max(voxel.max_traversability, cost);
                voxel.traversability_sum += cost;
                ++voxel.traversability_samples;
            }
        }

        for (const auto& item : voxels)
        {
            const ScanVoxel& voxel = item.second;
            std::vector<std::pair<uint32_t, uint32_t>> ordered_votes(
                voxel.semantic_votes.begin(), voxel.semantic_votes.end());
            std::sort(ordered_votes.begin(), ordered_votes.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return lhs.first < rhs.first;
                      });

            std::vector<octomap::SemanticObservation> observations;
            observations.reserve(ordered_votes.size());
            uint32_t dominant_label = invalid_semantic_label_;
            uint32_t dominant_votes = 0;
            for (const auto& vote : ordered_votes)
            {
                observations.emplace_back(toColor(kCityscapesPalette[vote.first]),
                                          vote.second);
                if (vote.second > dominant_votes)
                {
                    dominant_votes = vote.second;
                    dominant_label = vote.first;
                }
            }

            const octomap::ColorOcTreeNode::Color display_color =
                dominant_label < kCityscapesPalette.size()
                    ? toColor(kCityscapesPalette[dominant_label])
                    : octomap::ColorOcTreeNode::Color(160, 160, 160);
            tree_->updateEndpoint(
                voxel.key, observations, display_color,
                voxel.traversability_samples > 0, voxel.max_traversability,
                observation_confidence_, voxel.traversability_samples);
        }

        tree_->updateInnerOccupancy();
        if (enable_pruning_)
            tree_->prune();

        publishMaps(cloud->header.stamp);
        ROS_INFO_STREAM_THROTTLE(
            2.0, "traversability map input=" << point_count
            << ", aggregated voxels=" << voxels.size()
            << ", rejected=" << rejected_points
            << ", tree nodes=" << tree_->size());
    }

    bool validateFields(const sensor_msgs::PointCloud2& cloud) const
    {
        const bool valid =
            hasField(cloud, "x", sensor_msgs::PointField::FLOAT32) &&
            hasField(cloud, "y", sensor_msgs::PointField::FLOAT32) &&
            hasField(cloud, "z", sensor_msgs::PointField::FLOAT32) &&
            hasField(cloud, "traversability", sensor_msgs::PointField::FLOAT32) &&
            hasField(cloud, "semantic_lable", sensor_msgs::PointField::UINT32);
        if (!valid)
        {
            ROS_ERROR_THROTTLE(2.0,
                "/grids_points requires FLOAT32 x/y/z/traversability and UINT32 semantic_lable");
        }
        return valid;
    }

    void publishMaps(const ros::Time& stamp)
    {
        octomap_msgs::Octomap full_map;
        full_map.header.stamp = stamp;
        full_map.header.frame_id = world_frame_id_;
        tree_->setWriteExtendedData(true);
        if (octomap_msgs::fullMapToMsg(*tree_, full_map))
            full_map_pub_.publish(full_map);
        else
            ROS_ERROR("Failed to serialize SemanticTraversabilityOcTree");

        octomap_msgs::Octomap color_map;
        color_map.header = full_map.header;
        tree_->setWriteExtendedData(false);
        if (octomap_msgs::fullMapToMsg(*tree_, color_map))
        {
            color_map.id = "ColorOcTree";
            color_map_pub_.publish(color_map);
        }
        else
        {
            ROS_ERROR("Failed to serialize ColorOcTree compatibility map");
        }
        tree_->setWriteExtendedData(true);

        if (publish_voxel_clouds_)
            publishVoxelClouds(stamp);
        if (publish_traversability_2d_)
            publishTraversabilityMap(stamp);
    }

    void publishVoxelClouds(const ros::Time& stamp)
    {
        size_t occupied_leaf_count = 0;
        for (auto iterator = tree_->begin_leafs(), finish = tree_->end_leafs();
             iterator != finish; ++iterator)
        {
            if (tree_->isNodeOccupied(*iterator))
                ++occupied_leaf_count;
        }

        sensor_msgs::PointCloud2 combined;
        combined.header.stamp = stamp;
        combined.header.frame_id = world_frame_id_;
        sensor_msgs::PointCloud2Modifier modifier(combined);
        modifier.setPointCloud2Fields(
            12,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "semantic_label", 1, sensor_msgs::PointField::UINT32,
            "semantic_confidence", 1, sensor_msgs::PointField::FLOAT32,
            "measured_traversability", 1, sensor_msgs::PointField::FLOAT32,
            "max_traversability", 1, sensor_msgs::PointField::FLOAT32,
            "mean_traversability", 1, sensor_msgs::PointField::FLOAT32,
            "traversability_confidence", 1, sensor_msgs::PointField::FLOAT32,
            "observations", 1, sensor_msgs::PointField::UINT32,
            "traversability_valid", 1, sensor_msgs::PointField::UINT8,
            "valid_fraction", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(occupied_leaf_count);

        sensor_msgs::PointCloud2Iterator<float> x(combined, "x");
        sensor_msgs::PointCloud2Iterator<float> y(combined, "y");
        sensor_msgs::PointCloud2Iterator<float> z(combined, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> semantic_label(
            combined, "semantic_label");
        sensor_msgs::PointCloud2Iterator<float> semantic_confidence(
            combined, "semantic_confidence");
        sensor_msgs::PointCloud2Iterator<float> measured(
            combined, "measured_traversability");
        sensor_msgs::PointCloud2Iterator<float> maximum(
            combined, "max_traversability");
        sensor_msgs::PointCloud2Iterator<float> mean(
            combined, "mean_traversability");
        sensor_msgs::PointCloud2Iterator<float> traversal_confidence(
            combined, "traversability_confidence");
        sensor_msgs::PointCloud2Iterator<uint32_t> observations(
            combined, "observations");
        sensor_msgs::PointCloud2Iterator<uint8_t> valid(
            combined, "traversability_valid");
        sensor_msgs::PointCloud2Iterator<float> valid_fraction(
            combined, "valid_fraction");

        pcl::PointCloud<pcl::PointXYZRGB> semantic_visualization;
        pcl::PointCloud<pcl::PointXYZRGB> traversal_visualization;
        semantic_visualization.reserve(occupied_leaf_count);
        traversal_visualization.reserve(occupied_leaf_count);

        for (auto iterator = tree_->begin_leafs(), finish = tree_->end_leafs();
             iterator != finish; ++iterator)
        {
            if (!tree_->isNodeOccupied(*iterator))
                continue;
            const auto& state = iterator->getTraversability();
            const auto semantic_color = iterator->isSemanticsSet()
                ? iterator->getSemantics().getSemanticColor()
                : octomap::ColorOcTreeNode::Color(160, 160, 160);

            *x = iterator.getX();
            *y = iterator.getY();
            *z = iterator.getZ();
            *semantic_label = labelFromSemanticColor(semantic_color);
            *semantic_confidence = iterator->getSemanticConfidence();
            *measured = state.measured_cost;
            *maximum = state.max_cost;
            *mean = state.mean_cost;
            *traversal_confidence = state.confidence;
            *observations = state.observations;
            *valid = state.valid ? 1 : 0;
            *valid_fraction = state.valid_fraction;
            ++x; ++y; ++z; ++semantic_label; ++semantic_confidence;
            ++measured; ++maximum; ++mean; ++traversal_confidence; ++observations;
            ++valid; ++valid_fraction;

            pcl::PointXYZRGB semantic_point;
            semantic_point.x = iterator.getX();
            semantic_point.y = iterator.getY();
            semantic_point.z = iterator.getZ();
            semantic_point.r = semantic_color.r;
            semantic_point.g = semantic_color.g;
            semantic_point.b = semantic_color.b;
            semantic_visualization.push_back(semantic_point);

            pcl::PointXYZRGB traversal_point = semantic_point;
            if (state.valid)
            {
                traversal_point.r = static_cast<uint8_t>(255.0f * state.max_cost);
                traversal_point.g = static_cast<uint8_t>(255.0f * (1.0f - state.max_cost));
                traversal_point.b = 0;
            }
            else
            {
                traversal_point.r = traversal_point.g = traversal_point.b = 128;
            }
            traversal_visualization.push_back(traversal_point);
        }

        combined_cloud_pub_.publish(combined);

        sensor_msgs::PointCloud2 semantic_message;
        pcl::toROSMsg(semantic_visualization, semantic_message);
        semantic_message.header = combined.header;
        semantic_cloud_pub_.publish(semantic_message);

        sensor_msgs::PointCloud2 traversal_message;
        pcl::toROSMsg(traversal_visualization, traversal_message);
        traversal_message.header = combined.header;
        traversability_cloud_pub_.publish(traversal_message);
    }

    void publishTraversabilityMap(const ros::Time& stamp)
    {
        if (tree_->size() == 0)
            return;
        double min_x, min_y, min_z, max_x, max_y, max_z;
        tree_->getMetricMin(min_x, min_y, min_z);
        tree_->getMetricMax(max_x, max_y, max_z);
        const double resolution = tree_->getResolution();
        const uint32_t width =
            std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil((max_x - min_x) / resolution)));
        const uint32_t height =
            std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil((max_y - min_y) / resolution)));

        nav_msgs::OccupancyGrid map_template;
        map_template.header.stamp = stamp;
        map_template.header.frame_id = world_frame_id_;
        map_template.info.resolution = resolution;
        map_template.info.width = width;
        map_template.info.height = height;
        map_template.info.origin.position.x = min_x;
        map_template.info.origin.position.y = min_y;
        map_template.info.origin.orientation.w = 1.0;

        nav_msgs::OccupancyGrid occupancy_map = map_template;
        nav_msgs::OccupancyGrid maximum_map = map_template;
        nav_msgs::OccupancyGrid mean_map = map_template;
        const size_t cell_count = static_cast<size_t>(width) * height;
        occupancy_map.data.assign(cell_count, -1);
        maximum_map.data.assign(cell_count, -1);
        mean_map.data.assign(cell_count, -1);
        std::vector<double> mean_sums(cell_count, 0.0);
        std::vector<double> mean_weights(cell_count, 0.0);

        for (auto iterator = tree_->begin_leafs(), finish = tree_->end_leafs();
             iterator != finish; ++iterator)
        {
            const float half_size = 0.5f * iterator.getSize();
            const bool in_traversability_band =
                iterator.getZ() + half_size >= min_ground_z_ &&
                iterator.getZ() - half_size <= max_ground_z_;
            const bool in_occupancy_band =
                iterator.getZ() + half_size >= occupancy_min_z_ &&
                iterator.getZ() - half_size <= occupancy_max_z_;
            if (!in_traversability_band && !in_occupancy_band)
                continue;

            const int min_cell_x = std::max<int>(
                0, std::floor((iterator.getX() - half_size - min_x) / resolution));
            const int max_cell_x = std::min<int>(
                width - 1, std::ceil(
                    (iterator.getX() + half_size - min_x) / resolution) - 1);
            const int min_cell_y = std::max<int>(
                0, std::floor((iterator.getY() - half_size - min_y) / resolution));
            const int max_cell_y = std::min<int>(
                height - 1, std::ceil(
                    (iterator.getY() + half_size - min_y) / resolution) - 1);
            for (int cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y)
            {
                for (int cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x)
                {
                    const size_t cell = static_cast<size_t>(cell_y) * width + cell_x;
                    if (in_occupancy_band)
                    {
                        if (tree_->isNodeOccupied(*iterator))
                            occupancy_map.data[cell] = 100;
                        else if (occupancy_map.data[cell] < 0)
                            occupancy_map.data[cell] = 0;
                    }

                    const auto& state = iterator->getTraversability();
                    if (!in_traversability_band || !state.valid)
                        continue;
                    const int maximum = static_cast<int>(
                        std::round(100.0f * state.max_cost));
                    maximum_map.data[cell] = std::max<int>(
                        maximum_map.data[cell], maximum);
                    const double weight = std::max(0.0f, state.valid_fraction);
                    mean_sums[cell] += static_cast<double>(state.mean_cost) * weight;
                    mean_weights[cell] += weight;
                }
            }
        }
        for (size_t cell = 0; cell < cell_count; ++cell)
        {
            if (mean_weights[cell] > 0.0)
            {
                mean_map.data[cell] = static_cast<int8_t>(std::round(
                    100.0 * mean_sums[cell] / mean_weights[cell]));
            }
        }
        occupancy_map_pub_.publish(occupancy_map);
        traversability_map_pub_.publish(maximum_map);
        traversability_mean_map_pub_.publish(mean_map);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* cloud_subscriber_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter_;
    std::unique_ptr<octomap::SemanticTraversabilityOcTree> tree_;

    ros::Publisher full_map_pub_;
    ros::Publisher color_map_pub_;
    ros::Publisher traversability_cloud_pub_;
    ros::Publisher semantic_cloud_pub_;
    ros::Publisher combined_cloud_pub_;
    ros::Publisher occupancy_map_pub_;
    ros::Publisher traversability_map_pub_;
    ros::Publisher traversability_mean_map_pub_;
    ros::ServiceServer legacy_rle_service_;
    ros::ServiceServer traversability_rle_service_;
    ros::ServiceServer reset_service_;

    std::string input_topic_;
    std::string world_frame_id_;
    std::string save_path_;
    std::string load_path_;
    double resolution_;
    double max_range_;
    double clamping_thres_min_;
    double clamping_thres_max_;
    double occupancy_thres_;
    double prob_hit_;
    double prob_miss_;
    double semantic_phi_;
    double semantic_psi_;
    double rise_alpha_;
    double fall_alpha_;
    double merge_epsilon_;
    double confidence_merge_epsilon_;
    double observation_confidence_;
    bool enable_pruning_;
    bool publish_voxel_clouds_;
    bool publish_traversability_2d_;
    double min_ground_z_;
    double max_ground_z_;
    double occupancy_min_z_;
    double occupancy_max_z_;
    uint32_t invalid_semantic_label_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_traversability_octomap");
    SemanticTraversabilityOctomapNode node;
    ros::spin();
    node.save();
    return 0;
}
