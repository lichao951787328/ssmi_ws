#include <semantic_octomap_node/octomap_generator_ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <pcl/conversions.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy
#include <stdexcept>

OctomapGeneratorNode::OctomapGeneratorNode(ros::NodeHandle& nh): octomap_generator_(NULL), nh_(nh), floor_initialized_(false)
{
    // Initiate octree
    ROS_INFO("Semantic octomap generated!");
    toggle_color_service_ = nh_.advertiseService("toggle_use_semantic_color", &OctomapGeneratorNode::toggleUseSemanticColor, this);
    RLE_service_ = nh_.advertiseService("querry_RLE", &OctomapGeneratorNode::querry_RLE, this);
    reset_service_ = nh_.advertiseService("reset_semantic_octomap", &OctomapGeneratorNode::resetMap, this);

    reset();
    fullmap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    colormap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_color", 1, true);
    occ_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_map_2D", 1, true);
    pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, pointcloud_topic_, 5);
    tf_pointcloud_sub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointcloud_sub_, tf_listener_, world_frame_id_, 5);
    tf_pointcloud_sub_->registerCallback(boost::bind(&OctomapGeneratorNode::insertCloudCallback, this, _1));
}

OctomapGeneratorNode::~OctomapGeneratorNode()
{
    delete octomap_generator_;
}
/// Clear octomap and reset values to paramters from parameter server
void OctomapGeneratorNode::reset()
{
    delete octomap_generator_;
    octomap_generator_ = new OctomapGenerator<PCLSemantics, SemanticOctree>();

    nh_.getParam("/octomap/pointcloud_topic", pointcloud_topic_);
    nh_.getParam("/octomap/world_frame_id", world_frame_id_);
    nh_.param("/octomap/use_initial_pose_reference", use_initial_pose_reference_, false);
    nh_.param<std::string>("/octomap/reference_frame_id", reference_frame_id_, "map_start");
    nh_.param("/octomap/time_jump_reset_threshold", time_jump_reset_threshold_, 1.0);
    if (use_initial_pose_reference_ &&
        (reference_frame_id_.empty() || reference_frame_id_ == world_frame_id_))
    {
        throw std::runtime_error(
            "/octomap/reference_frame_id must differ from /octomap/world_frame_id");
    }
    have_initial_pose_reference_ = !use_initial_pose_reference_;
    initial_cloud_stamp_ = ros::Time();
    latest_cloud_stamp_ = ros::Time();
    initial_cloud_to_world_.setIdentity();
    nh_.getParam("/octomap/resolution", resolution_);
    nh_.getParam("/octomap/max_range", max_range_);
    nh_.getParam("/octomap/raycast_range", raycast_range_);
    nh_.param<std::string>("/octomap/input_mode", input_mode_, "sensor");
    const bool default_raycast_clearing = input_mode_ != "local_grid";
    nh_.param("/octomap/enable_raycast_clearing", enable_raycast_clearing_,
              default_raycast_clearing);
    if (!nh_.getParam("/octomap/obstacle_semantic_colors", obstacle_semantic_rgb_values_))
    {
        obstacle_semantic_rgb_values_ = {
            70, 70, 70, 102, 102, 156, 190, 153, 153, 153, 153, 153,
            250, 170, 30, 220, 220, 0, 107, 142, 35};
    }
    nh_.getParam("/octomap/clamping_thres_min", clamping_thres_min_);
    nh_.getParam("/octomap/clamping_thres_max", clamping_thres_max_);
    nh_.getParam("/octomap/occupancy_thres", occupancy_thres_);
    nh_.getParam("/octomap/prob_hit", prob_hit_);
    nh_.getParam("/octomap/prob_miss", prob_miss_);
    nh_.param("/octomap/dynamic_free_updates", dynamic_free_updates_, 64);
    nh_.param("/octomap/dynamic_free_confirmations", dynamic_free_confirmations_, 1);
    nh_.getParam("/octomap/psi", psi_);
    nh_.getParam("/octomap/phi", phi_);
    nh_.getParam("/octomap/publish_2d_map", publish_2d_map);
    nh_.getParam("/octomap/min_ground_z", min_ground_z);
    nh_.getParam("/octomap/max_ground_z", max_ground_z);
    nh_.param("/octomap/initialize_floor", initialize_floor_, false);
    nh_.param<std::string>("/octomap/initial_floor/robot_frame_id", initial_floor_robot_frame_id_, "base_link");
    nh_.param("/octomap/initial_floor/length", initial_floor_length_, 1.2);
    nh_.param("/octomap/initial_floor/width", initial_floor_width_, 0.8);
    nh_.param("/octomap/initial_floor/offset_x", initial_floor_offset_x_, 0.0);
    nh_.param("/octomap/initial_floor/offset_y", initial_floor_offset_y_, 0.0);
    nh_.param("/octomap/initial_floor/base_to_ground", initial_floor_base_to_ground_, 0.75);
    // When enabled, base_to_ground describes the physical floor surface and
    // the occupied voxel is placed half a resolution below it. The legacy
    // behavior treats the surface point as a voxel sample and remains the
    // default for existing robot configurations.
    nh_.param("/octomap/initial_floor/align_voxel_top", initial_floor_align_voxel_top_, false);
    nh_.param("/octomap/initial_floor/clear_above_floor", initial_floor_clear_above_, true);
    nh_.param("/octomap/initial_floor/clear_height", initial_floor_clear_height_, 1.0);
    if (!nh_.getParam("/octomap/initial_floor/semantic_rgb", initial_floor_semantic_rgb_))
        initial_floor_semantic_rgb_ = std::vector<int>{0, 0, 0};
    if (!nh_.getParam("/octomap/initial_floor/rgb", initial_floor_rgb_))
        initial_floor_rgb_ = std::vector<int>{204, 204, 204};
    if (initial_floor_semantic_rgb_.size() != 3 || initial_floor_rgb_.size() != 3)
    {
        ROS_WARN("Initial floor RGB parameters must contain exactly three values; using defaults");
        initial_floor_semantic_rgb_ = std::vector<int>{0, 0, 0};
        initial_floor_rgb_ = std::vector<int>{204, 204, 204};
    }
    floor_initialized_ = false;
    std::vector<uint32_t> obstacle_semantic_colors;
    if (obstacle_semantic_rgb_values_.size() % 3 != 0)
    {
        ROS_WARN("/octomap/obstacle_semantic_colors must be a flat list of RGB triples; ignoring it");
    }
    else
    {
        for (size_t i = 0; i < obstacle_semantic_rgb_values_.size(); i += 3)
        {
            const int r = obstacle_semantic_rgb_values_[i];
            const int g = obstacle_semantic_rgb_values_[i + 1];
            const int b = obstacle_semantic_rgb_values_[i + 2];
            if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
            {
                ROS_WARN("Ignoring out-of-range RGB triple in /octomap/obstacle_semantic_colors");
                continue;
            }
            obstacle_semantic_colors.push_back(
                (static_cast<uint32_t>(r) << 16) |
                (static_cast<uint32_t>(g) << 8) |
                static_cast<uint32_t>(b));
        }
    }
    octomap_generator_->setClampingThresMin(clamping_thres_min_);
    octomap_generator_->setClampingThresMax(clamping_thres_max_);
    octomap_generator_->setResolution(resolution_);
    octomap_generator_->setOccupancyThres(occupancy_thres_);
    octomap_generator_->setProbHit(prob_hit_);
    octomap_generator_->setProbMiss(prob_miss_);
    octomap_generator_->setDynamicFreeUpdates(dynamic_free_updates_);
    octomap_generator_->setDynamicFreeConfirmations(dynamic_free_confirmations_);
    octomap_generator_->setRaycastClearingEnabled(enable_raycast_clearing_);
    octomap_generator_->setObstacleSemanticColors(obstacle_semantic_colors);
    octomap_generator_->setPsi(psi_);
    octomap_generator_->setPhi(phi_);
    octomap_generator_->setRayCastRange(raycast_range_);
    octomap_generator_->setMaxRange(max_range_);

    if (input_mode_ != "sensor" && input_mode_ != "local_grid")
        ROS_WARN_STREAM("Unknown /octomap/input_mode '" << input_mode_
                        << "'; only raycast setting changes behavior");
    if (input_mode_ == "local_grid" && enable_raycast_clearing_)
        ROS_WARN("Raycast clearing is enabled for local_grid input; this can clear occluded map cells");
    ROS_INFO_STREAM("OctoMap input_mode=" << input_mode_
                    << ", raycast clearing="
                    << (enable_raycast_clearing_ ? "enabled" : "disabled")
                    << ", tracking frame=" << world_frame_id_
                    << ", map frame=" << mapFrameId());
}

const std::string& OctomapGeneratorNode::mapFrameId() const
{
    return use_initial_pose_reference_ ? reference_frame_id_ : world_frame_id_;
}

void OctomapGeneratorNode::captureInitialCloudReference(
    const tf::StampedTransform& sensor_to_world,
    const std_msgs::Header& cloud_header)
{
    initial_cloud_to_world_.setBasis(sensor_to_world.getBasis());
    initial_cloud_to_world_.setOrigin(sensor_to_world.getOrigin());
    initial_cloud_stamp_ = cloud_header.stamp;
    have_initial_pose_reference_ = true;

    geometry_msgs::TransformStamped reference_transform;
    reference_transform.header.stamp = initial_cloud_stamp_;
    reference_transform.header.frame_id = world_frame_id_;
    reference_transform.child_frame_id = reference_frame_id_;
    tf::transformTFToMsg(initial_cloud_to_world_, reference_transform.transform);
    reference_tf_broadcaster_.sendTransform(reference_transform);

    const double yaw = tf::getYaw(sensor_to_world.getRotation());
    ROS_INFO("Captured first semantic cloud at %.9f: %s <- %s, "
             "xyz=(%.6f, %.6f, %.6f), yaw=%.3f deg; octree frame is '%s'",
             initial_cloud_stamp_.toSec(), world_frame_id_.c_str(),
             cloud_header.frame_id.c_str(), sensor_to_world.getOrigin().x(),
             sensor_to_world.getOrigin().y(), sensor_to_world.getOrigin().z(),
             yaw * 180.0 / M_PI, reference_frame_id_.c_str());
}

bool OctomapGeneratorNode::resetMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    reset();
    ROS_INFO("Semantic octomap reset; initial floor will be recreated after the next point cloud");
    return true;
}

bool OctomapGeneratorNode::initializeRobotFloor(const ros::Time& stamp)
{
    if (!initialize_floor_ || floor_initialized_)
        return true;

    tf::StampedTransform base_to_world;
    try
    {
        tf_listener_.lookupTransform(world_frame_id_, initial_floor_robot_frame_id_, stamp, base_to_world);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(2.0, "Waiting to initialize floor: %s", ex.what());
        return false;
    }

    tf::Transform base_to_map;
    base_to_map.setBasis(base_to_world.getBasis());
    base_to_map.setOrigin(base_to_world.getOrigin());
    if (use_initial_pose_reference_)
    {
        if (!have_initial_pose_reference_)
            return false;
        base_to_map = initial_cloud_to_world_.inverse() * base_to_map;
    }

    SemanticOctree* tree = octomap_generator_->getOctree();
    const double resolution = tree->getResolution();
    const int cells_x = std::max(1, static_cast<int>(std::ceil(initial_floor_length_ / resolution)));
    const int cells_y = std::max(1, static_cast<int>(std::ceil(initial_floor_width_ / resolution)));
    const int clear_layers = initial_floor_clear_above_
        ? std::max(0, static_cast<int>(initial_floor_align_voxel_top_
            ? std::ceil(initial_floor_clear_height_ / resolution)
            : std::floor(initial_floor_clear_height_ / resolution)))
        : 0;
    const double floor_center_z = -initial_floor_base_to_ground_
        - (initial_floor_align_voxel_top_ ? 0.5 * resolution : 0.0);

    const octomap::ColorOcTreeNode::Color semantic_color(
        static_cast<uint8_t>(std::clamp(initial_floor_semantic_rgb_[0], 0, 255)),
        static_cast<uint8_t>(std::clamp(initial_floor_semantic_rgb_[1], 0, 255)),
        static_cast<uint8_t>(std::clamp(initial_floor_semantic_rgb_[2], 0, 255)));
    const octomap::ColorOcTreeNode::Color floor_color(
        static_cast<uint8_t>(std::clamp(initial_floor_rgb_[0], 0, 255)),
        static_cast<uint8_t>(std::clamp(initial_floor_rgb_[1], 0, 255)),
        static_cast<uint8_t>(std::clamp(initial_floor_rgb_[2], 0, 255)));

    size_t occupied_count = 0;
    size_t free_count = 0;
    for (int ix = 0; ix < cells_x; ++ix)
    {
        const double x = initial_floor_offset_x_ + (ix - 0.5 * (cells_x - 1)) * resolution;
        for (int iy = 0; iy < cells_y; ++iy)
        {
            const double y = initial_floor_offset_y_ + (iy - 0.5 * (cells_y - 1)) * resolution;
            // OctoMap stores voxel centers. In surface-aligned mode the cube's
            // upper face, rather than its center, follows the physical floor.
            // This prevents a 0.4 m floor voxel from occupying the Go2 body
            // space and avoids randomly selecting the layer above or below
            // zero when the base has a small roll or pitch.
            const tf::Vector3 local_floor(x, y, floor_center_z);
            const tf::Vector3 world_floor = base_to_map * local_floor;

            // Seed unknown nodes only. Existing sensor observations always win.
            if (tree->search(world_floor.x(), world_floor.y(), world_floor.z()) == NULL)
            {
                tree->updateNode(world_floor.x(), world_floor.y(), world_floor.z(), true,
                                 semantic_color, floor_color, false);
                ++occupied_count;
            }

            for (int layer = 1; layer <= clear_layers; ++layer)
            {
                const double free_center_z = initial_floor_align_voxel_top_
                    ? -initial_floor_base_to_ground_ + (layer - 0.5) * resolution
                    : -initial_floor_base_to_ground_ + layer * resolution;
                const tf::Vector3 local_free(x, y, free_center_z);
                const tf::Vector3 world_free = base_to_map * local_free;
                if (tree->search(world_free.x(), world_free.y(), world_free.z()) == NULL)
                {
                    tree->updateNode(world_free.x(), world_free.y(), world_free.z(), false);
                    ++free_count;
                }
            }
        }
    }

    floor_initialized_ = true;
    ROS_INFO("Initialized %.2f x %.2f m robot floor region: %zu occupied floor nodes, %zu free nodes (%s)",
             initial_floor_length_, initial_floor_width_, occupied_count, free_count,
             initial_floor_align_voxel_top_ ? "voxel top aligned to floor" : "legacy center sampling");
    return true;
}

bool OctomapGeneratorNode::toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());
    if(octomap_generator_->isUseSemanticColor())
        ROS_INFO("Using semantic color");
    else
        ROS_INFO("Using rgb color");

    full_map_msg_.header.frame_id = mapFrameId();
    full_map_msg_.header.stamp = latest_cloud_stamp_.isZero() ?
        ros::Time::now() : latest_cloud_stamp_;
    octomap_generator_->setWriteSemantics(true);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), full_map_msg_))
        fullmap_pub_.publish(full_map_msg_);
    else
        ROS_ERROR("Error serializing full OctoMap");

    color_map_msg_.header = full_map_msg_.header;
    octomap_generator_->setWriteSemantics(false);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), color_map_msg_))
    {
        // With semantics disabled the payload is byte-compatible with a
        // standard ColorOcTree, while the source tree still identifies itself
        // as SemanticOcTree. Keep RViz and non-semantic consumers compatible.
        color_map_msg_.id = "ColorOcTree";
        colormap_pub_.publish(color_map_msg_);
    }
    else
        ROS_ERROR("Error serializing color OctoMap");

    // Keep the tree in its native serialization mode outside publication.
    octomap_generator_->setWriteSemantics(true);

    return true;
}

bool OctomapGeneratorNode::querry_RLE(semantic_octomap::GetRLE::Request& request, semantic_octomap::GetRLE::Response& response)
{
    const octomap::point3d origin(request.origin.x, request.origin.y, request.origin.z);

    for (int i = 0; i < (int)request.endPoints.size(); ++i)
    {   
        const octomap::point3d endPoint(request.endPoints[i].x, request.endPoints[i].y, request.endPoints[i].z);
        semantic_octomap::RayRLE rayRLE_msg;
        if (octomap_generator_->get_ray_RLE(origin, endPoint, rayRLE_msg))
        {          
            response.RLE_list.push_back(rayRLE_msg);
        } 
    }
    
    return true;
}

void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if (cloud_msg->header.stamp.isZero())
    {
        ROS_ERROR_THROTTLE(2.0, "Semantic cloud has a zero acquisition timestamp");
        return;
    }
    if (!latest_cloud_stamp_.isZero() && cloud_msg->header.stamp < latest_cloud_stamp_)
    {
        const double rewind_seconds =
            (latest_cloud_stamp_ - cloud_msg->header.stamp).toSec();
        if (time_jump_reset_threshold_ >= 0.0 &&
            rewind_seconds > time_jump_reset_threshold_)
        {
            ROS_WARN("Semantic cloud time jumped backward by %.3f s; clearing the "
                     "octree and starting a new first-cloud mapping session",
                     rewind_seconds);
            reset();
        }
        else
        {
            ROS_WARN_THROTTLE(2.0,
                "Dropping out-of-order semantic cloud (stamp %.6f < latest %.6f)",
                cloud_msg->header.stamp.toSec(), latest_cloud_stamp_.toSec());
            return;
        }
    }

    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    if (cloud->width == 0 || cloud->height == 0 || cloud->data.empty())
    {
        ROS_WARN_THROTTLE(2.0, "Waiting for the first non-empty semantic cloud");
        return;
    }
    // Get tf transform
    tf::StampedTransform sensorToWorldTf;
    try
    {
        tf_listener_.lookupTransform(world_frame_id_, cloud_msg->header.frame_id, cloud_msg->header.stamp, sensorToWorldTf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }
    // Transform coordinate
    if (use_initial_pose_reference_ && !have_initial_pose_reference_)
        captureInitialCloudReference(sensorToWorldTf, cloud_msg->header);

    tf::Transform sensor_to_map;
    sensor_to_map.setBasis(sensorToWorldTf.getBasis());
    sensor_to_map.setOrigin(sensorToWorldTf.getOrigin());
    if (use_initial_pose_reference_)
        sensor_to_map = initial_cloud_to_world_.inverse() * sensor_to_map;

    Eigen::Matrix4f sensorToMap;
    pcl_ros::transformAsMatrix(sensor_to_map, sensorToMap);
    octomap_generator_->insertPointCloud(cloud, sensorToMap);
    initializeRobotFloor(cloud_msg->header.stamp);
    latest_cloud_stamp_ = cloud_msg->header.stamp;
    
    // Publish full octomap
    full_map_msg_.header.frame_id = mapFrameId();
    full_map_msg_.header.stamp = cloud_msg->header.stamp;

    octomap_generator_->setWriteSemantics(true);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), full_map_msg_))
        fullmap_pub_.publish(full_map_msg_);
    else
        ROS_ERROR("Error serializing full OctoMap");

    color_map_msg_.header = full_map_msg_.header;
    octomap_generator_->setWriteSemantics(false);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), color_map_msg_))
    {
        color_map_msg_.id = "ColorOcTree";
        colormap_pub_.publish(color_map_msg_);
    }
    else
        ROS_ERROR("Error serializing color OctoMap");

    octomap_generator_->setWriteSemantics(true);

    
    // Publish 2D occupancy map
    if (publish_2d_map)
        publish2DOccupancyMap(octomap_generator_->getOctree(), cloud_msg->header.stamp, mapFrameId());
}

void OctomapGeneratorNode::publish2DOccupancyMap(const SemanticOctree* octomap,
                                                 const ros::Time& stamp,
                                                 const std::string& frame_id)
{
  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);
  octomap::point3d minPt = octomap::point3d(minX, minY, minZ);

  unsigned int tree_depth = octomap->getTreeDepth();

  octomap::OcTreeKey paddedMinKey = octomap->coordToKey(minPt);

  nav_msgs::OccupancyGrid::Ptr occupancy_map (new nav_msgs::OccupancyGrid());

  unsigned int width, height;
  double res;

  unsigned int ds_shift = tree_depth-16;

  occupancy_map->header.stamp = stamp;
  occupancy_map->header.frame_id = frame_id;
  occupancy_map->info.resolution = res = octomap->getNodeSize(16);
  occupancy_map->info.width = width = (maxX-minX) / res + 1;
  occupancy_map->info.height = height = (maxY-minY) / res + 1;
  occupancy_map->info.origin.position.x = minX  - (res / (float)(1<<ds_shift) ) + res;
  occupancy_map->info.origin.position.y = minY  - (res / (float)(1<<ds_shift) );

  occupancy_map->data.clear();
  occupancy_map->data.resize(width*height, -1);

    // traverse all leafs in the tree:
  unsigned int treeDepth = std::min<unsigned int>(16, octomap->getTreeDepth());
  for (typename SemanticOctree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
  {
  
    double node_z = it.getZ();
    double node_half_side = pow(it.getSize(), 1/3) / 2;
    double top_side = node_z + node_half_side;
    double bottom_side = node_z - node_half_side;
    
    if((bottom_side >= min_ground_z && bottom_side <= max_ground_z) ||
       (top_side >= min_ground_z && top_side <= max_ground_z) ||
       (bottom_side <= min_ground_z && top_side >= max_ground_z))
    {
      bool occupied = octomap->isNodeOccupied(*it);
      int intSize = 1 << (16 - it.getDepth());

      octomap::OcTreeKey minKey=it.getIndexKey();

      for (int dx = 0; dx < intSize; dx++)
      {
        for (int dy = 0; dy < intSize; dy++)
        {
          int posX = std::max<int>(0, minKey[0] + dx - paddedMinKey[0]);
          posX>>=ds_shift;

          int posY = std::max<int>(0, minKey[1] + dy - paddedMinKey[1]);
          posY>>=ds_shift;

          int idx = width * posY + posX;

          if (occupied)
            occupancy_map->data[idx] = 100;
          else if (occupancy_map->data[idx] == -1)
          {
            occupancy_map->data[idx] = 0;
          }

        }
      }
    }
  }

  occ_map_pub_.publish(*occupancy_map);
}

bool OctomapGeneratorNode::save(const char* filename) const
{
    return octomap_generator_->save(filename);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_generator");
    ros::NodeHandle nh;
    OctomapGeneratorNode octomapGeneratorNode(nh);
    ros::spin();
    std::string save_path;
    nh.getParam("/octomap/save_path", save_path);

    octomapGeneratorNode.save(save_path.c_str());
    ROS_INFO("OctoMap saved.");
    return 0;
}
