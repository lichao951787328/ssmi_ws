#ifndef SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS_H
#define SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <semantic_octomap_node/octomap_generator.h>
#include <semantic_octomap_node/semantic_schema.h>
#include <std_srvs/Empty.h>
#include <semantic_octomap/GetRLE.h>
#include <semantic_octomap/RayRLE.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <cstdint>
#include <string>
#include <vector>
#include <octomap_msgs/Octomap.h>

class OctomapGeneratorNode{
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGeneratorNode(ros::NodeHandle& nh);
    /// Desturctor
    virtual ~OctomapGeneratorNode();
    /// Reset values to paramters from parameter server
    void reset();
    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void revokedFreeCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void revokedReclassifiedCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    
    void publish2DOccupancyMap(const SemanticOctree* octomap,
                               const ros::Time& stamp,
                               const std::string& frame_id);
    
    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    bool save(const char* filename) const;

protected:
    OctomapGeneratorBase<SemanticOctree>* octomap_generator_; ///<Octomap instance pointer
    ros::ServiceServer toggle_color_service_;  ///<ROS service to toggle semantic color display
    ros::ServiceServer RLE_service_;  ///<ROS service to querry RLE values
    ros::ServiceServer reset_service_; ///<ROS service to clear and reinitialize the semantic map
    bool toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); ///<Function to toggle whether write semantic color or rgb color as when serializing octree
    bool querry_RLE(semantic_octomap::GetRLE::Request& request, semantic_octomap::GetRLE::Response& response);
    bool resetMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool initializeRobotFloor(const ros::Time& stamp);
    const std::string& mapFrameId() const;
    void captureInitialCloudReference(const tf::StampedTransform& sensor_to_world,
                                      const std_msgs::Header& cloud_header);
    void transformFailureCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                                  tf::FilterFailureReason reason);
    void applyRevocationCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                              ros::Time& latest_stamp,
                              const char* reason);
    void publishMaps(const ros::Time& stamp);
    ros::NodeHandle nh_; ///<ROS handler
    ros::Publisher fullmap_pub_; ///<ROS publisher for full octomap message
    ros::Publisher colormap_pub_; ///<ROS publisher for color octomap message
    ros::Publisher occ_map_pub_; ///<ROS publisher for 2D occupancy map message
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pointcloud_sub_; ///<ROS subscriber for pointcloud message
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pointcloud_sub_; ///<ROS tf message filter to sychronize the tf and pointcloud messages
    ros::Subscriber revoked_free_sub_; ///<Explicit confirmed-static free revocations
    ros::Subscriber revoked_reclassified_sub_; ///<Explicit stable-reclassification revocations
    tf::TransformListener tf_listener_; ///<Listener for the transform between the camera and the world coordinates
    tf2_ros::StaticTransformBroadcaster reference_tf_broadcaster_;
    uint64_t tf_failure_count_; ///<Cumulative clouds rejected for unavailable/invalid TF
    std::string world_frame_id_; ///<Id of the world frame
    std::string reference_frame_id_; ///<Map frame whose origin is the first usable cloud pose
    std::string pointcloud_topic_; ///<Topic name for subscribed pointcloud message
    std::string revoked_free_topic_; ///<Upstream explicit-free revocation topic
    std::string revoked_reclassified_topic_; ///<Upstream stable-reclassification revocation topic
    bool enable_explicit_revocation_; ///<Whether explicit upstream revocations are consumed
    bool use_initial_pose_reference_; ///<Whether T0^-1*T(t) is used for insertion
    bool have_initial_pose_reference_; ///<Whether the first usable cloud established T0
    double time_jump_reset_threshold_; ///<Large rewind starts a new mapping session
    ros::Time initial_cloud_stamp_; ///<Sensor timestamp defining t0
    ros::Time latest_cloud_stamp_; ///<Latest cloud inserted into the current session
    ros::Time latest_revoked_free_stamp_; ///<Latest explicit-free event processed
    ros::Time latest_revoked_reclassified_stamp_; ///<Latest reclassification event processed
    tf::Transform initial_cloud_to_world_; ///<T_world_cloud(t0)
    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
    std::string input_mode_; ///<Raw sensor scan or already-fused local grid
    bool enable_raycast_clearing_; ///<Whether endpoint insertion clears sensor rays
    semantic_octomap::SemanticSchema semantic_schema_; ///<Shared semantic contract
    float clamping_thres_max_; ///<Upper bound of occupancy probability for a node
    float clamping_thres_min_; ///<Lower bound of occupancy probability for a node
    float psi_; ///<Increment update value for a semantic class
    float phi_; ///<Decrement update value for a semantic class
    float resolution_; ///<Resolution of octomap
    float occupancy_thres_; ///<Minimum occupancy probability for a node to be considered as occupied
    float prob_hit_;  ///<Hit probability of sensor
    float prob_miss_; ///<Miss probability of sensor
    int dynamic_free_updates_; ///<Maximum same-frame updates after an old dynamic voxel is observed free
    int dynamic_free_confirmations_; ///<Required consecutive free observations
    bool publish_2d_map;
    double min_ground_z;
    double max_ground_z;
    bool initialize_floor_;
    bool floor_initialized_;
    std::string initial_floor_robot_frame_id_;
    double initial_floor_length_;
    double initial_floor_width_;
    double initial_floor_offset_x_;
    double initial_floor_offset_y_;
    double initial_floor_base_to_ground_;
    bool initial_floor_align_voxel_top_;
    bool initial_floor_clear_above_;
    double initial_floor_clear_height_;
    std::vector<int> initial_floor_semantic_rgb_;
    std::vector<int> initial_floor_rgb_;
    octomap_msgs::Octomap full_map_msg_; ///<Complete semantic octomap message
    octomap_msgs::Octomap color_map_msg_; ///<ColorOcTree-compatible RViz message
};

#endif //SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS_H
