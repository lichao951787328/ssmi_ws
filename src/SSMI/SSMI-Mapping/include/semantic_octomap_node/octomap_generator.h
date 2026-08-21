#ifndef SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_H
#define SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>
#include <semantic_point_cloud/semantic_point_type.h>
#include <semantic_octomap_node/octomap_generator_base.h>
#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

typedef pcl::PointCloud<PointXYZRGBSemantic> PCLSemantics;
typedef octomap::SemanticOcTree<octomap::SemanticsLogOdds> SemanticOctree;
typedef octomap::SemanticOcTreeNode<octomap::SemanticsLogOdds> SemanticsOcTreeNode;

template<class CLOUD, class OCTREE>
class OctomapGenerator: public OctomapGeneratorBase<OCTREE>
{
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGenerator();

    virtual ~OctomapGenerator();

    virtual void setMaxRange(float max_range){max_range_ = max_range;}

    virtual void setRayCastRange(float raycast_range){raycast_range_ = raycast_range;}

    virtual void setRaycastClearingEnabled(bool enabled)
    {
        raycast_clearing_enabled_ = enabled;
    }

    virtual void setObstacleSemanticColors(const std::vector<uint32_t>& colors)
    {
        obstacle_semantic_colors_.clear();
        for (uint32_t color : colors)
            obstacle_semantic_colors_.insert(color & 0x00ffffffu);
    }

    virtual void setSemanticSchema(const semantic_octomap::SemanticSchema& schema)
    {
        semantic_schema_ = schema;
        use_semantic_schema_ = schema.isLoaded();
        obstacle_semantic_colors_.clear();
        for (const semantic_octomap::SemanticClass& semantic_class : schema.classes())
        {
            if (semantic_class.admit_to_global_map &&
                semantic_class.role == semantic_octomap::SemanticRole::StaticObstacle)
            {
                obstacle_semantic_colors_.insert(
                    semantic_octomap::SemanticSchema::packRgb(semantic_class.rgb));
            }
        }
    }

    virtual void setDynamicFreeUpdates(int updates)
    {
        dynamic_free_updates_ = std::max(0, updates);
    }

    virtual void setDynamicFreeConfirmations(int confirmations)
    {
        dynamic_free_confirmations_ = std::max(1, confirmations);
    }

    virtual void setClampingThresMin(float clamping_thres_min)
    {
        octomap_.setClampingThresMin(clamping_thres_min);
        octomap_.setMinLogOdds(clamping_thres_min);
    }

    virtual void setClampingThresMax(float clamping_thres_max)
    {
        octomap_.setClampingThresMax(clamping_thres_max);
        octomap_.setMaxLogOdds(clamping_thres_max);
    }

    virtual void setResolution(float resolution)
    {
        octomap_.setResolution(resolution);
    }

    virtual void setOccupancyThres(float occupancy_thres)
    {
        octomap_.setOccupancyThres(occupancy_thres);
    }

    virtual void setProbHit(float prob_hit)
    {
        octomap_.setProbHit(prob_hit);
    }

    virtual void setProbMiss(float prob_miss)
    {
        octomap_.setProbMiss(prob_miss);
    }

    /// Set phi, parameter for semantic octomap
    virtual void setPhi(float phi)
    {
        octomap_.setPhi(phi);
    }

    /// Set psi, parameter for semantic octomap
    virtual void setPsi(float psi)
    {
        octomap_.setPsi(psi);
    }

    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld);

    virtual std::size_t deleteVoxels(const std::vector<Eigen::Vector3f>& points);
    
    virtual bool get_ray_RLE(const octomap::point3d& origin, const octomap::point3d& end, semantic_octomap::RayRLE& rayRLE_msg);

    virtual void setUseSemanticColor(bool use);

    virtual bool isUseSemanticColor();
    
    virtual void setWriteSemantics(bool write);

    virtual bool doesWriteSemantics();

    virtual OCTREE* getOctree(){return &octomap_;}

    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    virtual bool save(const char* filename) const;

protected:
    OCTREE octomap_; ///<Templated octree instance
    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
    bool raycast_clearing_enabled_; ///<Whether inserted endpoints also clear free rays
    int dynamic_free_updates_; ///<Maximum same-frame updates for confirmed-free old dynamic voxels
    int dynamic_free_confirmations_; ///<Required consecutive free observations
    std::unordered_set<uint32_t> obstacle_semantic_colors_; ///<Static obstacle colors with voxel selection priority
    std::unordered_map<uint64_t, octomap::OcTreeKey> previous_dynamic_keys_; ///<Dynamic endpoints in the previous scan
    std::unordered_map<uint64_t, int> dynamic_free_confirmation_counts_; ///<Consecutive free evidence per dynamic voxel
    semantic_octomap::SemanticSchema semantic_schema_; ///<Shared runtime semantic contract
    bool use_semantic_schema_ = false; ///<Whether schema filtering/remapping is active

};

#endif //SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_H
