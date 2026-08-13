#ifndef SEMANTIC_OCTOMAP_SEMANTIC_TRAVERSABILITY_OCTREE_H
#define SEMANTIC_OCTOMAP_SEMANTIC_TRAVERSABILITY_OCTREE_H

#include <octomap/ColorOcTree.h>
#include <octomap/OccupancyOcTreeBase.h>

#include <semantic_octomap/RayRLE.h>
#include <semantic_octomap/TraversabilityRayRLE.h>
#include <semantic_octree/Semantics.h>

#include <cstdint>
#include <iosfwd>
#include <utility>
#include <vector>

namespace octomap
{
struct TraversabilityState
{
    float measured_cost;
    float confidence;
    uint32_t observations;
    bool valid;
    float max_cost;
    float mean_cost;
    float valid_fraction;

    TraversabilityState();

    bool operator==(const TraversabilityState& rhs) const;
    bool operator!=(const TraversabilityState& rhs) const { return !(*this == rhs); }
};

struct SemanticObservation
{
    ColorOcTreeNode::Color color;
    uint32_t votes;

    SemanticObservation(const ColorOcTreeNode::Color& color_value, uint32_t vote_count)
        : color(color_value), votes(vote_count) {}
};

class SemanticTraversabilityOcTree;

class SemanticTraversabilityOcTreeNode : public ColorOcTreeNode
{
public:
    friend class SemanticTraversabilityOcTree;

    SemanticTraversabilityOcTreeNode();
    SemanticTraversabilityOcTreeNode(const SemanticTraversabilityOcTreeNode& rhs);

    bool operator==(const SemanticTraversabilityOcTreeNode& rhs) const;
    void copyData(const SemanticTraversabilityOcTreeNode& from);

    const SemanticsLogOdds& getSemantics() const { return semantics_; }
    void setSemantics(const SemanticsLogOdds& semantics) { semantics_ = semantics; }
    bool isSemanticsSet() const { return semantics_.isSemanticsSet(); }

    const TraversabilityState& getTraversability() const { return traversability_; }
    void setTraversability(const TraversabilityState& state) { traversability_ = state; }

    float getSemanticConfidence() const;
    void updateSemanticsChildren();
    void updateTraversabilityChildren();

    std::istream& readData(std::istream& stream);
    std::ostream& writeData(std::ostream& stream) const;

private:
    SemanticsLogOdds fusedChildSemantics() const;
    SemanticsLogOdds semantics_;
    TraversabilityState traversability_;
    bool use_semantic_color_;
    bool write_extended_data_;
};

class SemanticTraversabilityOcTree
    : public OccupancyOcTreeBase<SemanticTraversabilityOcTreeNode>
{
public:
    using Node = SemanticTraversabilityOcTreeNode;
    using Base = OccupancyOcTreeBase<Node>;

    explicit SemanticTraversabilityOcTree(double resolution);
    SemanticTraversabilityOcTree* create() const override;
    std::string getTreeType() const override;

    void setPhi(float value) { phi_ = value; }
    void setPsi(float value) { psi_ = value; }
    void setSemanticMaxProbability(float value);
    void setSemanticMinProbability(float value);
    void setTraversabilityRiseAlpha(float value);
    void setTraversabilityFallAlpha(float value);
    void setTraversabilityMergeEpsilon(float value);
    void setConfidenceMergeEpsilon(float value);

    float getTraversabilityRiseAlpha() const { return rise_alpha_; }
    float getTraversabilityFallAlpha() const { return fall_alpha_; }
    float getTraversabilityMergeEpsilon() const { return merge_epsilon_; }

    Node* updateEndpoint(const OcTreeKey& key,
                         const std::vector<SemanticObservation>& semantic_observations,
                         const ColorOcTreeNode::Color& display_color,
                         bool traversability_valid,
                         float traversability,
                         float traversability_confidence,
                         uint32_t traversability_samples = 1);

    Node* updateFree(const OcTreeKey& key);

    void updateInnerOccupancy();
    bool pruneNode(Node* node) override;
    bool isNodeCollapsible(const Node* node) const override;

    void setUseSemanticColor(bool enabled);
    void setWriteExtendedData(bool enabled);

    bool getLegacyRayRLE(const point3d& origin, const point3d& end,
                         semantic_octomap::RayRLE& output);
    bool getTraversabilityRayRLE(
        const point3d& origin, const point3d& end,
        semantic_octomap::TraversabilityRayRLE& output);

private:
    void updateSemanticObservation(Node* node, const ColorOcTreeNode::Color& color);
    void updateTraversabilityObservation(Node* node, float measured,
                                         float confidence, uint32_t samples);
    void updateInnerOccupancyRecursive(Node* node, unsigned int depth);

    class StaticMemberInitializer
    {
    public:
        StaticMemberInitializer();
        void ensureLinking() const {}
    };
    static StaticMemberInitializer tree_type_initializer_;

    float phi_;
    float psi_;
    float semantic_max_log_odds_;
    float semantic_min_log_odds_;
    float rise_alpha_;
    float fall_alpha_;
    float merge_epsilon_;
    float confidence_merge_epsilon_;
};
}  // namespace octomap

#endif
