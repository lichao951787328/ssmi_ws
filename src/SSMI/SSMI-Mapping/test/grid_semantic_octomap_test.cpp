#include <gtest/gtest.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <semantic_octomap_node/grid_semantic_admission.h>
#include <semantic_octomap_node/octomap_generator.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <set>
#include <vector>

namespace
{
float packRgb(uint8_t r, uint8_t g, uint8_t b)
{
    const uint32_t bits = (static_cast<uint32_t>(r) << 16) |
                          (static_cast<uint32_t>(g) << 8) |
                          static_cast<uint32_t>(b);
    float packed = 0.0f;
    std::memcpy(&packed, &bits, sizeof(bits));
    return packed;
}

PointXYZRGBSemantic makePoint(float x, float y, float z,
                              uint8_t r, uint8_t g, uint8_t b)
{
    PointXYZRGBSemantic point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.data[3] = 1.0f;
    point.rgb = packRgb(r, g, b);
    point.semantic_color = point.rgb;
    return point;
}

pcl::PCLPointCloud2::Ptr toPcl2(const PCLSemantics& points)
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(points, *cloud);
    return cloud;
}
}  // namespace

TEST(GridSemanticAdmission, OnlyTerrainAndStaticLabelsAreAdmitted)
{
    for (uint32_t label = 0; label <= 18; ++label)
    {
        semantic_octomap::SemanticRgb color;
        const bool admitted = semantic_octomap::admittedSemanticColor(label, color);
        const bool expected = label == 0 || label == 1 ||
                              (label >= 2 && label <= 9);
        EXPECT_EQ(expected, admitted) << "label=" << label;
    }

    semantic_octomap::SemanticRgb color;
    EXPECT_FALSE(semantic_octomap::admittedSemanticColor(UINT32_MAX, color));
}

TEST(GridSemanticOctomap, StaticObstacleOutranksOrdinarySemanticInSameVoxel)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(5.0f);
    generator.setRaycastClearingEnabled(false);
    generator.setObstacleSemanticColors({0x66669cu});  // Cityscapes wall

    PCLSemantics cloud;
    // Road is closer to the origin, but wall must win the categorical voxel
    // selection because obstacle priority is evaluated before distance.
    cloud.push_back(makePoint(0.05f, 0.05f, 0.05f, 128, 64, 128));
    cloud.push_back(makePoint(0.15f, 0.05f, 0.05f, 102, 102, 156));

    generator.insertPointCloud(toPcl2(cloud), Eigen::Matrix4f::Identity());

    SemanticsOcTreeNode* node = generator.getOctree()->search(0.1, 0.05, 0.05);
    ASSERT_NE(nullptr, node);
    const octomap::ColorOcTreeNode::Color semantic = node->getSemantics().data[0].color;
    EXPECT_EQ(102, semantic.r);
    EXPECT_EQ(102, semantic.g);
    EXPECT_EQ(156, semantic.b);
}

TEST(GridSemanticOctomap, DisabledRaycastDoesNotCreateFreeCellsAlongLocalGridRay)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.2f);
    generator.setMaxRange(5.0f);
    generator.setRayCastRange(5.0f);
    generator.setRaycastClearingEnabled(false);

    PCLSemantics cloud;
    cloud.push_back(makePoint(1.05f, 0.05f, 0.05f, 102, 102, 156));
    generator.insertPointCloud(toPcl2(cloud), Eigen::Matrix4f::Identity());

    EXPECT_EQ(nullptr, generator.getOctree()->search(0.45, 0.05, 0.05));
    SemanticsOcTreeNode* endpoint = generator.getOctree()->search(1.05, 0.05, 0.05);
    ASSERT_NE(nullptr, endpoint);
    EXPECT_TRUE(generator.getOctree()->isNodeOccupied(endpoint));
}

TEST(GridSemanticOctomap, DynamicOcclusionDoesNotReplaceConfirmedStaticObstacle)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(5.0f);
    generator.setRaycastClearingEnabled(false);
    generator.setObstacleSemanticColors({0x66669cu});

    PCLSemantics static_cloud;
    static_cloud.push_back(makePoint(0.1f, 0.1f, 0.1f, 102, 102, 156));
    generator.insertPointCloud(toPcl2(static_cloud), Eigen::Matrix4f::Identity());

    PCLSemantics dynamic_occluder;
    dynamic_occluder.push_back(makePoint(0.1f, 0.1f, 0.1f, 255, 0, 255));
    generator.insertPointCloud(toPcl2(dynamic_occluder), Eigen::Matrix4f::Identity());

    SemanticsOcTreeNode* node = generator.getOctree()->search(0.1, 0.1, 0.1);
    ASSERT_NE(nullptr, node);
    const octomap::ColorOcTreeNode::Color semantic = node->getSemantics().data[0].color;
    EXPECT_EQ(102, semantic.r);
    EXPECT_EQ(102, semantic.g);
    EXPECT_EQ(156, semantic.b);
    EXPECT_TRUE(generator.getOctree()->isNodeOccupied(node));
}

TEST(GridSemanticOctomap, FullMapContainsOnlyAdmittedSemanticClasses)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(20.0f);
    generator.setRaycastClearingEnabled(false);
    generator.setObstacleSemanticColors({
        0x464646u, 0x66669cu, 0xbe9999u, 0x999999u,
        0xfaaa1eu, 0xdcdc00u, 0x6b8e23u});

    PCLSemantics admitted_cloud;
    size_t admitted_count = 0;
    for (uint32_t label = 0; label <= 18; ++label)
    {
        semantic_octomap::SemanticRgb color;
        if (!semantic_octomap::admittedSemanticColor(label, color))
            continue;
        admitted_cloud.push_back(makePoint(
            0.2f + static_cast<float>(admitted_count), 0.2f, 0.2f,
            color[0], color[1], color[2]));
        ++admitted_count;
    }
    generator.insertPointCloud(toPcl2(admitted_cloud), Eigen::Matrix4f::Identity());

    octomap_msgs::Octomap full_map;
    generator.setWriteSemantics(true);
    ASSERT_TRUE(octomap_msgs::fullMapToMsg(*generator.getOctree(), full_map));
    EXPECT_EQ("SemanticOcTree", full_map.id);
    EXPECT_NEAR(0.4, full_map.resolution, 1e-6);

    std::unique_ptr<octomap::AbstractOcTree> decoded_base(
        octomap_msgs::fullMsgToMap(full_map));
    SemanticOctree* decoded = dynamic_cast<SemanticOctree*>(decoded_base.get());
    ASSERT_NE(nullptr, decoded);

    std::set<uint32_t> allowed_colors;
    for (uint32_t label = 0; label <= 9; ++label)
    {
        semantic_octomap::SemanticRgb color;
        if (semantic_octomap::admittedSemanticColor(label, color))
        {
            allowed_colors.insert((static_cast<uint32_t>(color[0]) << 16) |
                                  (static_cast<uint32_t>(color[1]) << 8) |
                                  static_cast<uint32_t>(color[2]));
        }
    }

    size_t leaf_count = 0;
    for (SemanticOctree::leaf_iterator it = decoded->begin_leafs(),
         end = decoded->end_leafs(); it != end; ++it)
    {
        const octomap::ColorOcTreeNode::Color semantic =
            it->getSemantics().data[0].color;
        const uint32_t bits = (static_cast<uint32_t>(semantic.r) << 16) |
                              (static_cast<uint32_t>(semantic.g) << 8) |
                              static_cast<uint32_t>(semantic.b);
        EXPECT_NE(allowed_colors.end(), allowed_colors.find(bits));
        EXPECT_NE(0xff00ffu, bits);
        ++leaf_count;
    }
    EXPECT_EQ(admitted_count, leaf_count);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
