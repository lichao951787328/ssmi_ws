#include <gtest/gtest.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <semantic_octomap_node/octomap_generator.h>

#include <cstdint>
#include <cstring>
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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
