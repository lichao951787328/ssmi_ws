#include <gtest/gtest.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <semantic_octomap_node/octomap_generator.h>
#include <semantic_octomap_node/semantic_schema.h>

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

XmlRpc::XmlRpcValue rgb(int r, int g, int b)
{
    XmlRpc::XmlRpcValue value;
    value.setSize(3);
    value[0] = r;
    value[1] = g;
    value[2] = b;
    return value;
}

XmlRpc::XmlRpcValue semanticClass(
    int label, const char* name, const XmlRpc::XmlRpcValue& color,
    const char* role, bool global_map)
{
    XmlRpc::XmlRpcValue value;
    value["label"] = label;
    value["name"] = name;
    value["meaning"] = std::string("test ") + name;
    value["rgb"] = color;
    value["role"] = role;
    value["global_map"] = global_map;
    return value;
}

XmlRpc::XmlRpcValue schemaRoot()
{
    XmlRpc::XmlRpcValue root;
    root["version"] = 1;
    root["input"]["label_field"] = "semantic_lable";
    root["input"]["traversability_field"] = "traversability";
    root["defaults"]["initial_floor_label"] = 0;
    root["unknown"]["policy"] = "exclude";
    root["unknown"]["fallback_label"] = 3;
    root["unknown"]["name"] = "unknown";
    root["unknown"]["meaning"] = "not configured";
    root["unknown"]["rgb"] = rgb(127, 127, 127);
    root["traversability"]["override_semantics"] = true;
    root["traversability"]["obstacle_threshold"] = 0.75;
    root["traversability"]["obstacle_label"] = 3;

    XmlRpc::XmlRpcValue classes;
    classes.setSize(6);
    classes[0] = semanticClass(
        0, "road", rgb(128, 64, 128), "terrain", true);
    classes[1] = semanticClass(
        2, "building", rgb(70, 70, 70), "static_obstacle", true);
    classes[2] = semanticClass(
        3, "wall", rgb(102, 102, 156), "static_obstacle", true);
    classes[3] = semanticClass(
        9, "terrain", rgb(152, 251, 152), "terrain", true);
    classes[4] = semanticClass(
        10, "sky", rgb(70, 130, 180), "ignore", false);
    classes[5] = semanticClass(
        11, "person", rgb(220, 20, 60), "dynamic_obstacle", false);
    XmlRpc::XmlRpcValue aliases;
    aliases.setSize(1);
    aliases[0] = rgb(255, 0, 255);
    classes[5]["input_rgb_aliases"] = aliases;
    root["classes"] = classes;
    return root;
}

semantic_octomap::SemanticSchema testSchema()
{
    return semantic_octomap::SemanticSchema::fromXmlRpc(schemaRoot());
}
}  // namespace

TEST(SemanticSchema, RolesAndGlobalAdmissionComeFromConfiguration)
{
    const semantic_octomap::SemanticSchema schema = testSchema();
    EXPECT_TRUE(schema.resolveLabel(0).admitted);
    EXPECT_EQ(semantic_octomap::SemanticRole::Terrain,
              schema.resolveLabel(0).role);
    EXPECT_TRUE(schema.resolveLabel(2).admitted);
    EXPECT_EQ(semantic_octomap::SemanticRole::StaticObstacle,
              schema.resolveLabel(2).role);
    EXPECT_FALSE(schema.resolveLabel(10).admitted);
    EXPECT_EQ(semantic_octomap::SemanticRole::Ignore,
              schema.resolveLabel(10).role);
    EXPECT_FALSE(schema.resolveLabel(11).admitted);
    EXPECT_EQ(semantic_octomap::SemanticRole::DynamicObstacle,
              schema.resolveLabel(11).role);
    EXPECT_FALSE(schema.resolveLabel(UINT32_MAX).known);
    EXPECT_FALSE(schema.resolveLabel(UINT32_MAX).admitted);
}

TEST(SemanticSchema, HighCostOverrideNeverPromotesDynamicOrUnknown)
{
    const semantic_octomap::SemanticSchema schema = testSchema();
    const semantic_octomap::ResolvedSemantic high_cost_road =
        schema.resolveLabel(0, true);
    EXPECT_TRUE(high_cost_road.admitted);
    EXPECT_EQ(3u, high_cost_road.label);
    EXPECT_EQ((semantic_octomap::SemanticRgb{{102, 102, 156}}),
              high_cost_road.rgb);

    const semantic_octomap::ResolvedSemantic dynamic =
        schema.resolveLabel(11, true);
    EXPECT_TRUE(dynamic.known);
    EXPECT_FALSE(dynamic.admitted);
    EXPECT_EQ(11u, dynamic.label);

    const semantic_octomap::ResolvedSemantic unknown =
        schema.resolveLabel(UINT32_MAX, true);
    EXPECT_FALSE(unknown.known);
    EXPECT_FALSE(unknown.admitted);
}

TEST(SemanticSchema, InputColorAliasResolvesToCanonicalDynamicClass)
{
    const semantic_octomap::ResolvedSemantic dynamic =
        testSchema().resolveColor(0x00ff00ffu);
    EXPECT_TRUE(dynamic.known);
    EXPECT_EQ(11u, dynamic.label);
    EXPECT_EQ((semantic_octomap::SemanticRgb{{220, 20, 60}}), dynamic.rgb);
    EXPECT_EQ(semantic_octomap::SemanticRole::DynamicObstacle, dynamic.role);
    EXPECT_FALSE(dynamic.admitted);
}

TEST(SemanticSchema, UnknownCanBeExplicitlyMappedToFallback)
{
    XmlRpc::XmlRpcValue root = schemaRoot();
    root["unknown"]["policy"] = "map_to_fallback";
    const semantic_octomap::ResolvedSemantic unknown =
        semantic_octomap::SemanticSchema::fromXmlRpc(root).resolveLabel(999u);
    EXPECT_TRUE(unknown.known);
    EXPECT_TRUE(unknown.admitted);
    EXPECT_EQ(3u, unknown.label);
}

TEST(SemanticSchema, DynamicClassCannotBeConfiguredIntoPersistentMap)
{
    XmlRpc::XmlRpcValue root = schemaRoot();
    root["classes"][5]["global_map"] = true;
    EXPECT_THROW(semantic_octomap::SemanticSchema::fromXmlRpc(root),
                 std::runtime_error);
}

TEST(GridSemanticOctomap, StaticObstacleOutranksOrdinarySemanticInSameVoxel)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(5.0f);
    generator.setRaycastClearingEnabled(false);
    generator.setSemanticSchema(testSchema());

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
    generator.setSemanticSchema(testSchema());

    PCLSemantics cloud;
    cloud.push_back(makePoint(1.05f, 0.05f, 0.05f, 102, 102, 156));
    generator.insertPointCloud(toPcl2(cloud), Eigen::Matrix4f::Identity());

    EXPECT_EQ(nullptr, generator.getOctree()->search(0.45, 0.05, 0.05));
    SemanticsOcTreeNode* endpoint = generator.getOctree()->search(1.05, 0.05, 0.05);
    ASSERT_NE(nullptr, endpoint);
    EXPECT_TRUE(generator.getOctree()->isNodeOccupied(endpoint));
}

TEST(GridSemanticOctomap, DynamicInputDoesNotReplacePersistentStaticObstacle)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(5.0f);
    generator.setRaycastClearingEnabled(false);
    generator.setSemanticSchema(testSchema());

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

TEST(GridSemanticOctomap, DynamicInputDoesNotReplacePersistentTerrain)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(5.0f);
    generator.setRaycastClearingEnabled(false);
    generator.setSemanticSchema(testSchema());

    PCLSemantics terrain;
    terrain.push_back(makePoint(0.1f, 0.1f, 0.1f, 128, 64, 128));
    generator.insertPointCloud(toPcl2(terrain), Eigen::Matrix4f::Identity());

    PCLSemantics dynamic;
    dynamic.push_back(makePoint(0.1f, 0.1f, 0.1f, 255, 0, 255));
    generator.insertPointCloud(toPcl2(dynamic), Eigen::Matrix4f::Identity());

    SemanticsOcTreeNode* node = generator.getOctree()->search(0.1, 0.1, 0.1);
    ASSERT_NE(nullptr, node);
    const octomap::ColorOcTreeNode::Color semantic =
        node->getSemantics().data[0].color;
    EXPECT_EQ(128, semantic.r);
    EXPECT_EQ(64, semantic.g);
    EXPECT_EQ(128, semantic.b);
}

TEST(GridSemanticOctomap, ExplicitRevocationDeletesOnlyAddressedVoxelAndIsIdempotent)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(5.0f);
    generator.setRaycastClearingEnabled(false);
    generator.setSemanticSchema(testSchema());

    PCLSemantics cloud;
    cloud.push_back(makePoint(0.1f, 0.1f, 0.1f, 102, 102, 156));
    cloud.push_back(makePoint(0.5f, 0.1f, 0.1f, 102, 102, 156));
    generator.insertPointCloud(toPcl2(cloud), Eigen::Matrix4f::Identity());
    ASSERT_NE(nullptr, generator.getOctree()->search(0.1, 0.1, 0.1));
    ASSERT_NE(nullptr, generator.getOctree()->search(0.5, 0.1, 0.1));

    const std::vector<Eigen::Vector3f> revoked{
        Eigen::Vector3f(0.1f, 0.1f, 0.1f),
        Eigen::Vector3f(0.1f, 0.1f, 0.1f)};
    EXPECT_EQ(1u, generator.deleteVoxels(revoked));
    EXPECT_EQ(nullptr, generator.getOctree()->search(0.1, 0.1, 0.1));
    EXPECT_NE(nullptr, generator.getOctree()->search(0.5, 0.1, 0.1));

    EXPECT_EQ(0u, generator.deleteVoxels(revoked));
    EXPECT_EQ(nullptr, generator.getOctree()->search(0.1, 0.1, 0.1));
    EXPECT_NE(nullptr, generator.getOctree()->search(0.5, 0.1, 0.1));
}

TEST(GridSemanticOctomap, FullMapContainsOnlyAdmittedSemanticClasses)
{
    OctomapGenerator<PCLSemantics, SemanticOctree> generator;
    generator.setResolution(0.4f);
    generator.setMaxRange(20.0f);
    generator.setRaycastClearingEnabled(false);
    const semantic_octomap::SemanticSchema schema = testSchema();
    generator.setSemanticSchema(schema);

    PCLSemantics input_cloud;
    size_t admitted_count = 0;
    size_t input_count = 0;
    std::set<uint32_t> allowed_colors;
    for (const semantic_octomap::SemanticClass& semantic_class : schema.classes())
    {
        const semantic_octomap::SemanticRgb& color = semantic_class.rgb;
        input_cloud.push_back(makePoint(
            0.2f + 0.8f * static_cast<float>(input_count), 0.2f, 0.2f,
            color[0], color[1], color[2]));
        ++input_count;
        if (semantic_class.admit_to_global_map)
        {
            allowed_colors.insert(
                semantic_octomap::SemanticSchema::packRgb(color));
            ++admitted_count;
        }
    }
    input_cloud.push_back(makePoint(
        0.2f + 0.8f * static_cast<float>(input_count), 0.2f, 0.2f,
        1, 2, 3));
    generator.insertPointCloud(toPcl2(input_cloud), Eigen::Matrix4f::Identity());

    octomap_msgs::Octomap full_map;
    generator.setWriteSemantics(true);
    ASSERT_TRUE(octomap_msgs::fullMapToMsg(*generator.getOctree(), full_map));
    EXPECT_EQ("SemanticOcTree", full_map.id);
    EXPECT_NEAR(0.4, full_map.resolution, 1e-6);

    std::unique_ptr<octomap::AbstractOcTree> decoded_base(
        octomap_msgs::fullMsgToMap(full_map));
    SemanticOctree* decoded = dynamic_cast<SemanticOctree*>(decoded_base.get());
    ASSERT_NE(nullptr, decoded);

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
        EXPECT_NE(0x00ff00ffu, bits);
        ++leaf_count;
    }
    EXPECT_EQ(admitted_count, leaf_count);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
