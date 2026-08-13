#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <semantic_point_cloud/semantic_point_type.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_set>
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

Rgb readRgbParam(ros::NodeHandle& private_nh,
                 const std::string& name,
                 const Rgb& fallback)
{
    std::vector<int> values;
    if (!private_nh.getParam(name, values))
        return fallback;

    if (values.size() != 3 ||
        std::any_of(values.begin(), values.end(),
                    [](int value) { return value < 0 || value > 255; }))
    {
        ROS_WARN_STREAM("~" << name
                        << " must contain exactly three values in [0, 255]; using default");
        return fallback;
    }

    return Rgb{{static_cast<uint8_t>(values[0]),
                static_cast<uint8_t>(values[1]),
                static_cast<uint8_t>(values[2])}};
}

std::unordered_set<uint32_t> readLabelSet(ros::NodeHandle& private_nh,
                                          const std::string& name,
                                          const std::vector<int>& fallback)
{
    std::vector<int> values;
    if (!private_nh.getParam(name, values))
        values = fallback;

    std::unordered_set<uint32_t> labels;
    for (int value : values)
    {
        if (value < 0)
        {
            ROS_WARN_STREAM("Ignoring negative label " << value << " in ~" << name);
            continue;
        }
        labels.insert(static_cast<uint32_t>(value));
    }
    return labels;
}
}  // namespace

class GridSemanticAdapter
{
public:
    GridSemanticAdapter()
        : nh_(), private_nh_("~")
    {
        private_nh_.param<std::string>("input_topic", input_topic_, "/grids_points");
        private_nh_.param<std::string>("output_topic", output_topic_,
                                       "/semantic_pcl/semantic_pcl");
        private_nh_.param("obstacle_threshold", obstacle_threshold_, 0.6);

        int invalid_label = -1;
        private_nh_.param("invalid_semantic_label", invalid_label, -1);
        invalid_semantic_label_ = static_cast<uint32_t>(invalid_label);

        int fallback_label = 3;
        private_nh_.param("fallback_obstacle_label", fallback_label, 3);
        if (fallback_label < 0 ||
            fallback_label >= static_cast<int>(kCityscapesPalette.size()))
        {
            ROS_WARN("~fallback_obstacle_label is outside the Cityscapes palette; using wall (3)");
            fallback_label = 3;
        }
        fallback_obstacle_label_ = static_cast<uint32_t>(fallback_label);

        traversable_labels_ = readLabelSet(private_nh_, "traversable_labels", {0, 1, 9});
        existing_obstacle_labels_ = readLabelSet(
            private_nh_, "existing_obstacle_labels", {2, 3, 4, 5, 6, 7, 8});
        dynamic_labels_ = readLabelSet(
            private_nh_, "dynamic_labels", {11, 12, 13, 14, 15, 16, 17, 18});

        dynamic_rgb_ = readRgbParam(private_nh_, "dynamic_rgb", Rgb{{255, 0, 255}});
        unknown_rgb_ = readRgbParam(private_nh_, "unknown_rgb", Rgb{{255, 255, 255}});

        output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        input_sub_ = nh_.subscribe(input_topic_, 1, &GridSemanticAdapter::cloudCallback, this);

        ROS_INFO_STREAM("Grid semantic adapter: " << input_topic_ << " -> " << output_topic_
                        << ", traversability obstacle threshold=" << obstacle_threshold_);
    }

private:
    Rgb mapSemantic(uint32_t semantic_label, float traversability,
                    bool& remapped_high_cost) const
    {
        remapped_high_cost = false;

        if (dynamic_labels_.count(semantic_label) != 0)
            return dynamic_rgb_;

        if (existing_obstacle_labels_.count(semantic_label) != 0 &&
            semantic_label < kCityscapesPalette.size())
            return kCityscapesPalette[semantic_label];

        const bool high_cost = std::isfinite(traversability) &&
                               traversability >= obstacle_threshold_;
        const bool is_unknown = semantic_label == invalid_semantic_label_ ||
                                semantic_label >= kCityscapesPalette.size();
        if (high_cost &&
            (is_unknown || traversable_labels_.count(semantic_label) != 0))
        {
            remapped_high_cost = true;
            return kCityscapesPalette[fallback_obstacle_label_];
        }

        if (!is_unknown)
            return kCityscapesPalette[semantic_label];

        return unknown_rgb_;
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        const bool fields_valid =
            hasField(*input, "x", sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, "y", sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, "z", sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, "traversability", sensor_msgs::PointField::FLOAT32) &&
            hasField(*input, "semantic_lable", sensor_msgs::PointField::UINT32);
        if (!fields_valid)
        {
            ROS_ERROR_THROTTLE(2.0,
                "/grids_points must contain FLOAT32 x/y/z/traversability and UINT32 semantic_lable fields");
            return;
        }

        pcl::PointCloud<PointXYZRGBSemantic> output;
        pcl_conversions::toPCL(input->header, output.header);
        output.reserve(static_cast<size_t>(input->width) * input->height);
        output.is_dense = true;

        sensor_msgs::PointCloud2ConstIterator<float> x_it(*input, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_it(*input, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_it(*input, "z");
        sensor_msgs::PointCloud2ConstIterator<float> traversability_it(*input, "traversability");
        sensor_msgs::PointCloud2ConstIterator<uint32_t> semantic_it(*input, "semantic_lable");

        const size_t point_count = static_cast<size_t>(input->width) * input->height;
        size_t high_cost_remapped = 0;
        size_t invalid_xyz = 0;
        for (size_t i = 0; i < point_count;
             ++i, ++x_it, ++y_it, ++z_it, ++traversability_it, ++semantic_it)
        {
            if (!std::isfinite(*x_it) || !std::isfinite(*y_it) || !std::isfinite(*z_it))
            {
                ++invalid_xyz;
                continue;
            }

            bool remapped = false;
            const Rgb semantic_rgb = mapSemantic(*semantic_it, *traversability_it, remapped);
            if (remapped)
                ++high_cost_remapped;

            PointXYZRGBSemantic point;
            point.x = *x_it;
            point.y = *y_it;
            point.z = *z_it;
            point.data[3] = 1.0f;
            const float packed_rgb = packRgb(semantic_rgb);
            point.rgb = packed_rgb;
            point.semantic_color = packed_rgb;
            output.push_back(point);
        }

        output.width = static_cast<uint32_t>(output.size());
        output.height = 1;

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(output, output_msg);
        output_msg.header = input->header;
        output_pub_.publish(output_msg);

        ROS_INFO_STREAM_THROTTLE(2.0,
            "grid adapter input=" << point_count << ", output=" << output.size()
            << ", high-cost remapped=" << high_cost_remapped
            << ", invalid xyz skipped=" << invalid_xyz);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    std::string input_topic_;
    std::string output_topic_;
    double obstacle_threshold_;
    uint32_t invalid_semantic_label_;
    uint32_t fallback_obstacle_label_;
    std::unordered_set<uint32_t> traversable_labels_;
    std::unordered_set<uint32_t> existing_obstacle_labels_;
    std::unordered_set<uint32_t> dynamic_labels_;
    Rgb dynamic_rgb_;
    Rgb unknown_rgb_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_semantic_adapter");
    GridSemanticAdapter adapter;
    ros::spin();
    return 0;
}
