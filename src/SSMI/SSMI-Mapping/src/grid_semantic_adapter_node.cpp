#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <semantic_point_cloud/semantic_point_type.h>
#include <semantic_octomap_node/semantic_schema.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace
{
using Rgb = semantic_octomap::SemanticRgb;

float packRgb(const Rgb& color)
{
    const uint32_t bits = (static_cast<uint32_t>(color[0]) << 16) |
                          (static_cast<uint32_t>(color[1]) << 8) |
                          static_cast<uint32_t>(color[2]);
    float packed = 0.0f;
    std::memcpy(&packed, &bits, sizeof(bits));
    return packed;
}

bool hasField(const sensor_msgs::PointCloud2& cloud,
              const std::string& name,
              uint8_t datatype)
{
    for (const sensor_msgs::PointField& field : cloud.fields)
    {
        if (field.name == name)
            return field.datatype == datatype;
    }
    return false;
}

}  // namespace

class GridSemanticAdapter
{
public:
    GridSemanticAdapter()
        : nh_(), private_nh_("~"), received_clouds_(0), received_points_(0),
          received_static_(0), received_terrain_(0), admitted_static_(0),
          admitted_terrain_(0), high_cost_obstacles_(0), rejected_dynamic_(0),
          rejected_ignored_(0), rejected_unknown_(0), invalid_xyz_(0)
    {
        schema_ = semantic_octomap::SemanticSchema::fromRos(nh_);
        private_nh_.param<std::string>(
            "input_topic", input_topic_,
            "/local_3d_semantic_voxel_map/global_semantic_admission_grid");
        private_nh_.param<std::string>("output_topic", output_topic_,
                                       "/semantic_pcl/global_admitted");

        output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        input_sub_ = nh_.subscribe(input_topic_, 1, &GridSemanticAdapter::cloudCallback, this);

        ROS_INFO_STREAM("Grid semantic adapter: " << input_topic_ << " -> " << output_topic_
                        << "; trusting the upstream non-dynamic local snapshot, "
                        << "semantic schema classes=" << schema_.classes().size()
                        << ", high-cost threshold=" << schema_.obstacleThreshold());
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        const bool fields_valid =
            hasField(*input, "x", sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, "y", sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, "z", sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, schema_.traversabilityField(),
                     sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, schema_.labelField(),
                     sensor_msgs::PointField::UINT32);
        if (!fields_valid)
        {
            ROS_ERROR_THROTTLE(2.0,
                "Local semantic admission grid fields do not match semantic_schema/input");
            return;
        }

        ++received_clouds_;

        pcl::PointCloud<PointXYZRGBSemantic> output;
        pcl_conversions::toPCL(input->header, output.header);
        output.reserve(static_cast<size_t>(input->width) * input->height);
        output.is_dense = true;

        sensor_msgs::PointCloud2ConstIterator<float> x_it(*input, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_it(*input, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_it(*input, "z");
        sensor_msgs::PointCloud2ConstIterator<float> traversability_it(
            *input, schema_.traversabilityField());
        sensor_msgs::PointCloud2ConstIterator<uint32_t> semantic_it(
            *input, schema_.labelField());

        const size_t point_count = static_cast<size_t>(input->width) * input->height;
        received_points_ += point_count;
        for (size_t i = 0; i < point_count;
             ++i, ++x_it, ++y_it, ++z_it, ++traversability_it, ++semantic_it)
        {
            const semantic_octomap::ResolvedSemantic base_semantic =
                schema_.resolveLabel(*semantic_it);
            if (base_semantic.known &&
                base_semantic.role == semantic_octomap::SemanticRole::StaticObstacle)
                ++received_static_;
            else if (base_semantic.known &&
                     base_semantic.role == semantic_octomap::SemanticRole::Terrain)
                ++received_terrain_;

            if (!std::isfinite(*x_it) || !std::isfinite(*y_it) || !std::isfinite(*z_it))
            {
                ++invalid_xyz_;
                continue;
            }

            const bool high_cost = std::isfinite(*traversability_it) &&
                *traversability_it >= schema_.obstacleThreshold();
            const semantic_octomap::ResolvedSemantic resolved =
                schema_.resolveLabel(*semantic_it, high_cost);
            if (!resolved.admitted)
            {
                if (!resolved.known)
                    ++rejected_unknown_;
                else if (resolved.role ==
                         semantic_octomap::SemanticRole::DynamicObstacle)
                    ++rejected_dynamic_;
                else
                    ++rejected_ignored_;
                continue;
            }
            const Rgb semantic_rgb = resolved.rgb;
            if (high_cost)
                ++high_cost_obstacles_;

            PointXYZRGBSemantic point;
            point.x = *x_it;
            point.y = *y_it;
            point.z = *z_it;
            point.data[3] = 1.0f;
            const float packed_rgb = packRgb(semantic_rgb);
            point.rgb = packed_rgb;
            point.semantic_color = packed_rgb;
            output.push_back(point);

            if (resolved.role == semantic_octomap::SemanticRole::StaticObstacle)
                ++admitted_static_;
            else if (resolved.role == semantic_octomap::SemanticRole::Terrain)
                ++admitted_terrain_;
        }

        output.width = static_cast<uint32_t>(output.size());
        output.height = 1;

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(output, output_msg);
        output_msg.header = input->header;
        output_pub_.publish(output_msg);

        ROS_INFO_STREAM_THROTTLE(2.0,
            "global admission cumulative: clouds=" << received_clouds_
            << ", received points=" << received_points_
            << ", static received/admitted=" << received_static_ << "/" << admitted_static_
            << ", terrain received/admitted=" << received_terrain_ << "/" << admitted_terrain_
            << ", high-cost obstacle encoded=" << high_cost_obstacles_
            << ", rejected dynamic/ignore/unknown=" << rejected_dynamic_ << "/"
            << rejected_ignored_ << "/" << rejected_unknown_
            << ", invalid xyz dropped=" << invalid_xyz_);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    std::string input_topic_;
    std::string output_topic_;
    semantic_octomap::SemanticSchema schema_;
    uint64_t received_clouds_;
    uint64_t received_points_;
    uint64_t received_static_;
    uint64_t received_terrain_;
    uint64_t admitted_static_;
    uint64_t admitted_terrain_;
    uint64_t high_cost_obstacles_;
    uint64_t rejected_dynamic_;
    uint64_t rejected_ignored_;
    uint64_t rejected_unknown_;
    uint64_t invalid_xyz_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_semantic_adapter");
    GridSemanticAdapter adapter;
    ros::spin();
    return 0;
}
