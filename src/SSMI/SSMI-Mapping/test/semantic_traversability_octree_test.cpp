#include <gtest/gtest.h>

#include <octomap/AbstractOcTree.h>
#include <octomap/ColorOcTree.h>

#include <semantic_octree/SemanticOcTree.h>
#include <semantic_traversability_octree/SemanticTraversabilityOcTree.h>

#include <array>
#include <cmath>
#include <memory>
#include <sstream>
#include <vector>

namespace
{
using Tree = octomap::SemanticTraversabilityOcTree;
using Node = octomap::SemanticTraversabilityOcTreeNode;

const octomap::ColorOcTreeNode::Color kRoad(128, 64, 128);
const octomap::ColorOcTreeNode::Color kWall(102, 102, 156);

std::vector<octomap::SemanticObservation> semanticVotes(
    const octomap::ColorOcTreeNode::Color& color, uint32_t votes = 1)
{
    return {octomap::SemanticObservation(color, votes)};
}

octomap::OcTreeKey keyAt(Tree& tree, double x, double y, double z)
{
    octomap::OcTreeKey key;
    EXPECT_TRUE(tree.coordToKeyChecked(x, y, z, key));
    return key;
}

std::array<octomap::OcTreeKey, 8> siblingLeafKeys(Tree& tree)
{
    octomap::OcTreeKey base = keyAt(tree, 0.05, 0.05, 0.05);
    base[0] = static_cast<octomap::key_type>(base[0] & ~1u);
    base[1] = static_cast<octomap::key_type>(base[1] & ~1u);
    base[2] = static_cast<octomap::key_type>(base[2] & ~1u);
    std::array<octomap::OcTreeKey, 8> keys;
    for (size_t index = 0; index < keys.size(); ++index)
    {
        keys[index] = base;
        keys[index][0] += (index & 1u) != 0;
        keys[index][1] += (index & 2u) != 0;
        keys[index][2] += (index & 4u) != 0;
    }
    return keys;
}

void observe(Tree& tree, const octomap::OcTreeKey& key, float cost,
             const octomap::ColorOcTreeNode::Color& semantic = kRoad)
{
    tree.updateEndpoint(key, semanticVotes(semantic), semantic, true, cost, 1.0f, 1);
}
}  // namespace

TEST(SemanticTraversabilityOctree, SameVoxelKeepsSemanticVoteAndMaximumRisk)
{
    Tree tree(0.2);
    const octomap::OcTreeKey key = keyAt(tree, 0.1, 0.1, 0.1);
    tree.updateEndpoint(
        key,
        {octomap::SemanticObservation(kRoad, 3),
         octomap::SemanticObservation(kWall, 1)},
        kRoad, true, 0.9f, 1.0f, 4);

    const Node* node = tree.search(key);
    ASSERT_NE(nullptr, node);
    EXPECT_EQ(kRoad, node->getSemantics().data[0].color);
    EXPECT_FLOAT_EQ(0.9f, node->getTraversability().measured_cost);
    EXPECT_FLOAT_EQ(0.9f, node->getTraversability().max_cost);
    EXPECT_EQ(4u, node->getTraversability().observations);
}

TEST(SemanticTraversabilityOctree, DangerRisesFasterThanSafetyFalls)
{
    Tree tree(0.2);
    tree.setTraversabilityRiseAlpha(0.70f);
    tree.setTraversabilityFallAlpha(0.15f);
    const octomap::OcTreeKey key = keyAt(tree, 0.1, 0.1, 0.1);

    observe(tree, key, 0.1f);
    observe(tree, key, 0.9f);
    ASSERT_NE(nullptr, tree.search(key));
    EXPECT_NEAR(0.66f, tree.search(key)->getTraversability().measured_cost, 1e-6f);

    observe(tree, key, 0.1f);
    EXPECT_NEAR(0.576f, tree.search(key)->getTraversability().measured_cost, 1e-6f);
}

TEST(SemanticTraversabilityOctree, UnknownIsDifferentFromMeasuredZero)
{
    Tree tree(0.2);
    const octomap::OcTreeKey unknown_key = keyAt(tree, 0.1, 0.1, 0.1);
    const octomap::OcTreeKey zero_key = keyAt(tree, 0.3, 0.1, 0.1);
    tree.updateEndpoint(unknown_key, semanticVotes(kRoad), kRoad, false, 0.0f, 0.0f, 0);
    observe(tree, zero_key, 0.0f);

    ASSERT_NE(nullptr, tree.search(unknown_key));
    ASSERT_NE(nullptr, tree.search(zero_key));
    EXPECT_FALSE(tree.search(unknown_key)->getTraversability().valid);
    EXPECT_TRUE(tree.search(zero_key)->getTraversability().valid);
    EXPECT_FLOAT_EQ(0.0f, tree.search(zero_key)->getTraversability().measured_cost);
}

TEST(SemanticTraversabilityOctree, ParentSummarizesRiskWithoutPruningHeterogeneousChildren)
{
    Tree tree(0.1);
    tree.setTraversabilityMergeEpsilon(0.05f);
    const auto keys = siblingLeafKeys(tree);
    for (size_t index = 0; index < keys.size(); ++index)
        observe(tree, keys[index], index == 7 ? 0.9f : 0.1f);
    tree.updateInnerOccupancy();

    const Node* parent = tree.search(keys[0], tree.getTreeDepth() - 1);
    ASSERT_NE(nullptr, parent);
    EXPECT_FLOAT_EQ(0.9f, parent->getTraversability().max_cost);
    EXPECT_NEAR(0.2f, parent->getTraversability().mean_cost, 1e-6f);
    EXPECT_FLOAT_EQ(1.0f, parent->getTraversability().valid_fraction);

    const size_t leaves_before = tree.getNumLeafNodes();
    tree.prune();
    EXPECT_EQ(leaves_before, tree.getNumLeafNodes());
    EXPECT_EQ(8u, tree.getNumLeafNodes());
}

TEST(SemanticTraversabilityOctree, SimilarCostsPruneAndExpandWithBoundedInheritance)
{
    Tree tree(0.1);
    tree.setTraversabilityMergeEpsilon(0.05f);
    const auto keys = siblingLeafKeys(tree);
    for (size_t index = 0; index < keys.size(); ++index)
        observe(tree, keys[index], 0.20f + 0.004f * index);
    tree.updateInnerOccupancy();
    tree.prune();
    ASSERT_EQ(1u, tree.getNumLeafNodes());

    const float inherited_mean = tree.search(keys[0])->getTraversability().mean_cost;
    observe(tree, keys[7], 0.9f);
    ASSERT_EQ(8u, tree.getNumLeafNodes());
    const Node* sibling = tree.search(keys[0]);
    ASSERT_NE(nullptr, sibling);
    EXPECT_NEAR(inherited_mean, sibling->getTraversability().measured_cost, 1e-6f);
    EXPECT_LE(std::fabs(inherited_mean - 0.20f), tree.getTraversabilityMergeEpsilon());
}

TEST(SemanticTraversabilityOctree, FreeUpdateDoesNotSetOrClearTraversability)
{
    Tree tree(0.2);
    const octomap::OcTreeKey surface_key = keyAt(tree, 0.1, 0.1, 0.1);
    const octomap::OcTreeKey free_key = keyAt(tree, 0.3, 0.1, 0.1);
    observe(tree, surface_key, 0.8f);
    tree.updateFree(surface_key);
    tree.updateFree(free_key);

    ASSERT_NE(nullptr, tree.search(surface_key));
    ASSERT_NE(nullptr, tree.search(free_key));
    EXPECT_FLOAT_EQ(0.8f, tree.search(surface_key)->getTraversability().measured_cost);
    EXPECT_TRUE(tree.search(surface_key)->getTraversability().valid);
    EXPECT_FALSE(tree.search(free_key)->getTraversability().valid);
}

TEST(SemanticTraversabilityOctree, SaveLoadPreservesNewTypeAndState)
{
    Tree tree(0.2);
    const octomap::OcTreeKey key = keyAt(tree, 0.1, 0.1, 0.1);
    observe(tree, key, 0.73f, kWall);
    tree.updateInnerOccupancy();

    std::stringstream stream;
    ASSERT_TRUE(tree.write(stream));
    std::unique_ptr<octomap::AbstractOcTree> loaded(octomap::AbstractOcTree::read(stream));
    Tree* loaded_tree = dynamic_cast<Tree*>(loaded.get());
    ASSERT_NE(nullptr, loaded_tree);
    const Node* loaded_node = loaded_tree->search(key);
    ASSERT_NE(nullptr, loaded_node);
    EXPECT_EQ(kWall, loaded_node->getSemantics().data[0].color);
    EXPECT_FLOAT_EQ(0.73f, loaded_node->getTraversability().measured_cost);
    EXPECT_EQ(1u, loaded_node->getTraversability().observations);
}

TEST(SemanticTraversabilityOctree, ColorCompatibilityPayloadLoadsAsStandardColorTree)
{
    Tree tree(0.2);
    const octomap::OcTreeKey key = keyAt(tree, 0.1, 0.1, 0.1);
    observe(tree, key, 0.73f, kWall);
    tree.updateInnerOccupancy();
    tree.setWriteExtendedData(false);

    std::stringstream stream;
    tree.writeData(stream);
    octomap::ColorOcTree color_tree(0.2);
    color_tree.readData(stream);
    const octomap::ColorOcTreeNode* node = color_tree.search(key);
    ASSERT_NE(nullptr, node);
    EXPECT_EQ(kWall, node->getColor());
    EXPECT_TRUE(color_tree.isNodeOccupied(*node));
}

TEST(SemanticTraversabilityOctree, LegacySemanticTreeFormatStillLoads)
{
    using LegacyTree = octomap::SemanticOcTree<octomap::SemanticsLogOdds>;
    LegacyTree legacy(0.2);
    legacy.setPhi(-0.1f);
    legacy.setPsi(1.0f);
    legacy.setMinLogOdds(0.0001f);
    legacy.setMaxLogOdds(0.99f);
    legacy.updateNode(0.1, 0.1, 0.1, true, kRoad, kRoad, false);

    std::stringstream stream;
    ASSERT_TRUE(legacy.write(stream));
    std::unique_ptr<octomap::AbstractOcTree> loaded(octomap::AbstractOcTree::read(stream));
    ASSERT_NE(nullptr, dynamic_cast<LegacyTree*>(loaded.get()));
    EXPECT_EQ("SemanticOcTree", loaded->getTreeType());
}

TEST(SemanticTraversabilityOctree, LegacyAndVersionedRleStaySeparate)
{
    Tree tree(0.2);
    observe(tree, keyAt(tree, 0.3, 0.1, 0.1), 0.8f, kWall);

    semantic_octomap::RayRLE legacy;
    ASSERT_TRUE(tree.getLegacyRayRLE(
        octomap::point3d(0.1f, 0.1f, 0.1f),
        octomap::point3d(0.7f, 0.1f, 0.1f), legacy));
    ASSERT_FALSE(legacy.le_list.empty());
    for (const semantic_octomap::LE& entry : legacy.le_list)
        EXPECT_EQ(5u, entry.le.size());

    semantic_octomap::TraversabilityRayRLE versioned;
    ASSERT_TRUE(tree.getTraversabilityRayRLE(
        octomap::point3d(0.1f, 0.1f, 0.1f),
        octomap::point3d(0.7f, 0.1f, 0.1f), versioned));
    ASSERT_FALSE(versioned.entries.empty());
    for (const semantic_octomap::TraversabilityLE& entry : versioned.entries)
        EXPECT_EQ(semantic_octomap::TraversabilityLE::CURRENT_VERSION,
                  entry.version);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
