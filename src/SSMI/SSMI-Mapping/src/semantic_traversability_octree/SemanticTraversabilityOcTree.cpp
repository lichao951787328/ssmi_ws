#include <semantic_traversability_octree/SemanticTraversabilityOcTree.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>

namespace octomap
{
namespace
{
constexpr float kUnknownSemanticLogOdds = -0.1f;

float clampUnit(float value)
{
    return std::max(0.0f, std::min(1.0f, value));
}

std::array<double, 4> semanticLogOdds(const SemanticTraversabilityOcTreeNode* node)
{
    std::array<double, 4> values;
    if (node == nullptr)
    {
        values.fill(kUnknownSemanticLogOdds);
    }
    else if (!node->isSemanticsSet())
    {
        values.fill(node->getLogOdds() - std::log(4.0));
    }
    else
    {
        const SemanticsLogOdds& semantics = node->getSemantics();
        for (size_t index = 0; index < 3; ++index)
            values[index] = semantics.data[index].logOdds;
        values[3] = semantics.others;
    }
    return values;
}

bool sameLegacyEntry(const semantic_octomap::LE& lhs,
                     const std::array<double, 4>& rhs)
{
    if (lhs.le.size() != 5)
        return false;
    for (size_t index = 0; index < 4; ++index)
    {
        if (lhs.le[index + 1] != rhs[index])
            return false;
    }
    return true;
}

semantic_octomap::TraversabilityLE makeTraversabilityEntry(
    const SemanticTraversabilityOcTreeNode* node)
{
    semantic_octomap::TraversabilityLE entry;
    entry.version = semantic_octomap::TraversabilityLE::CURRENT_VERSION;
    entry.run_length = 1;
    const std::array<double, 4> semantic_values = semanticLogOdds(node);
    for (size_t index = 0; index < semantic_values.size(); ++index)
        entry.semantic_log_odds[index] = semantic_values[index];

    if (node != nullptr)
    {
        const TraversabilityState& state = node->getTraversability();
        entry.measured_traversability = state.measured_cost;
        entry.max_traversability = state.max_cost;
        entry.mean_traversability = state.mean_cost;
        entry.traversability_confidence = state.confidence;
        entry.valid_fraction = state.valid_fraction;
        entry.observations = state.observations;
        entry.valid = state.valid;
    }
    else
    {
        entry.measured_traversability = 0.0f;
        entry.max_traversability = 0.0f;
        entry.mean_traversability = 0.0f;
        entry.traversability_confidence = 0.0f;
        entry.valid_fraction = 0.0f;
        entry.observations = 0;
        entry.valid = false;
    }
    return entry;
}

bool sameTraversabilityEntry(const semantic_octomap::TraversabilityLE& lhs,
                             const semantic_octomap::TraversabilityLE& rhs)
{
    if (lhs.version != rhs.version)
        return false;
    for (size_t index = 0; index < 4; ++index)
    {
        if (lhs.semantic_log_odds[index] != rhs.semantic_log_odds[index])
            return false;
    }
    return lhs.measured_traversability == rhs.measured_traversability &&
           lhs.max_traversability == rhs.max_traversability &&
           lhs.mean_traversability == rhs.mean_traversability &&
           lhs.traversability_confidence == rhs.traversability_confidence &&
           lhs.valid_fraction == rhs.valid_fraction &&
           lhs.observations == rhs.observations && lhs.valid == rhs.valid;
}
}  // namespace

TraversabilityState::TraversabilityState()
    : measured_cost(0.0f), confidence(0.0f), observations(0), valid(false),
      max_cost(0.0f), mean_cost(0.0f), valid_fraction(0.0f)
{
}

bool TraversabilityState::operator==(const TraversabilityState& rhs) const
{
    return measured_cost == rhs.measured_cost && confidence == rhs.confidence &&
           observations == rhs.observations && valid == rhs.valid &&
           max_cost == rhs.max_cost && mean_cost == rhs.mean_cost &&
           valid_fraction == rhs.valid_fraction;
}

SemanticTraversabilityOcTreeNode::SemanticTraversabilityOcTreeNode()
    : ColorOcTreeNode(), semantics_(), traversability_(),
      use_semantic_color_(true), write_extended_data_(true)
{
}

SemanticTraversabilityOcTreeNode::SemanticTraversabilityOcTreeNode(
    const SemanticTraversabilityOcTreeNode& rhs)
{
    copyData(rhs);
}

bool SemanticTraversabilityOcTreeNode::operator==(
    const SemanticTraversabilityOcTreeNode& rhs) const
{
    return value == rhs.value && semantics_ == rhs.semantics_ &&
           traversability_ == rhs.traversability_;
}

void SemanticTraversabilityOcTreeNode::copyData(
    const SemanticTraversabilityOcTreeNode& from)
{
    ColorOcTreeNode::copyData(from);
    semantics_ = from.semantics_;
    traversability_ = from.traversability_;
    use_semantic_color_ = from.use_semantic_color_;
    write_extended_data_ = from.write_extended_data_;
}

float SemanticTraversabilityOcTreeNode::getSemanticConfidence() const
{
    if (!isSemanticsSet())
        return 0.0f;
    double denominator = std::exp(semantics_.others);
    for (size_t index = 0; index < 3; ++index)
    {
        if (semantics_.data[index].color != Color(255, 255, 255))
            denominator += std::exp(semantics_.data[index].logOdds);
    }
    return denominator > 0.0
               ? static_cast<float>(std::exp(semantics_.data[0].logOdds) / denominator)
               : 0.0f;
}

SemanticsLogOdds SemanticTraversabilityOcTreeNode::fusedChildSemantics() const
{
    SemanticsLogOdds output;
    bool started = false;
    if (children == nullptr)
        return output;

    for (size_t index = 0; index < 8; ++index)
    {
        const SemanticTraversabilityOcTreeNode* child =
            static_cast<const SemanticTraversabilityOcTreeNode*>(children[index]);
        if (child == nullptr || !child->isSemanticsSet())
            continue;
        output = started ? SemanticsLogOdds::semanticFusion(output, child->semantics_)
                         : child->semantics_;
        started = true;
    }
    return output;
}

void SemanticTraversabilityOcTreeNode::updateSemanticsChildren()
{
    semantics_ = fusedChildSemantics();
}

void SemanticTraversabilityOcTreeNode::updateTraversabilityChildren()
{
    TraversabilityState summary;
    if (children == nullptr)
    {
        traversability_ = summary;
        return;
    }

    float valid_weight = 0.0f;
    double mean_sum = 0.0;
    double confidence_sum = 0.0;
    double observation_sum = 0.0;
    float maximum = 0.0f;
    for (size_t index = 0; index < 8; ++index)
    {
        const SemanticTraversabilityOcTreeNode* child =
            static_cast<const SemanticTraversabilityOcTreeNode*>(children[index]);
        if (child == nullptr || !child->traversability_.valid)
            continue;

        const TraversabilityState& state = child->traversability_;
        const float weight = clampUnit(state.valid_fraction);
        if (weight <= 0.0f)
            continue;
        valid_weight += weight;
        mean_sum += static_cast<double>(state.mean_cost) * weight;
        confidence_sum += static_cast<double>(state.confidence) * weight;
        observation_sum += static_cast<double>(state.observations) * weight;
        maximum = std::max(maximum, state.max_cost);
    }

    if (valid_weight > 0.0f)
    {
        summary.valid = true;
        summary.valid_fraction = clampUnit(valid_weight / 8.0f);
        summary.max_cost = maximum;
        summary.mean_cost = static_cast<float>(mean_sum / valid_weight);
        summary.measured_cost = summary.mean_cost;
        summary.confidence = static_cast<float>(confidence_sum / valid_weight);
        summary.observations = static_cast<uint32_t>(
            std::min<double>(std::numeric_limits<uint32_t>::max(),
                             std::round(observation_sum / valid_weight)));
    }
    traversability_ = summary;
}

std::istream& SemanticTraversabilityOcTreeNode::readData(std::istream& stream)
{
    stream.read(reinterpret_cast<char*>(&value), sizeof(value));
    stream.read(reinterpret_cast<char*>(&color), sizeof(Color));
    stream.read(reinterpret_cast<char*>(&semantics_), sizeof(SemanticsLogOdds));
    stream.read(reinterpret_cast<char*>(&traversability_.measured_cost), sizeof(float));
    stream.read(reinterpret_cast<char*>(&traversability_.confidence), sizeof(float));
    stream.read(reinterpret_cast<char*>(&traversability_.observations), sizeof(uint32_t));
    uint8_t valid = 0;
    stream.read(reinterpret_cast<char*>(&valid), sizeof(valid));
    traversability_.valid = valid != 0;
    stream.read(reinterpret_cast<char*>(&traversability_.max_cost), sizeof(float));
    stream.read(reinterpret_cast<char*>(&traversability_.mean_cost), sizeof(float));
    stream.read(reinterpret_cast<char*>(&traversability_.valid_fraction), sizeof(float));
    return stream;
}

std::ostream& SemanticTraversabilityOcTreeNode::writeData(std::ostream& stream) const
{
    stream.write(reinterpret_cast<const char*>(&value), sizeof(value));
    const Color output_color = use_semantic_color_ && isSemanticsSet()
                                   ? semantics_.getSemanticColor() : color;
    stream.write(reinterpret_cast<const char*>(&output_color), sizeof(Color));
    if (!write_extended_data_)
        return stream;

    stream.write(reinterpret_cast<const char*>(&semantics_), sizeof(SemanticsLogOdds));
    stream.write(reinterpret_cast<const char*>(&traversability_.measured_cost), sizeof(float));
    stream.write(reinterpret_cast<const char*>(&traversability_.confidence), sizeof(float));
    stream.write(reinterpret_cast<const char*>(&traversability_.observations), sizeof(uint32_t));
    const uint8_t valid = traversability_.valid ? 1 : 0;
    stream.write(reinterpret_cast<const char*>(&valid), sizeof(valid));
    stream.write(reinterpret_cast<const char*>(&traversability_.max_cost), sizeof(float));
    stream.write(reinterpret_cast<const char*>(&traversability_.mean_cost), sizeof(float));
    stream.write(reinterpret_cast<const char*>(&traversability_.valid_fraction), sizeof(float));
    return stream;
}

SemanticTraversabilityOcTree::StaticMemberInitializer::StaticMemberInitializer()
{
    SemanticTraversabilityOcTree* tree = new SemanticTraversabilityOcTree(0.1);
    tree->clearKeyRays();
    AbstractOcTree::registerTreeType(tree);
}

SemanticTraversabilityOcTree::StaticMemberInitializer
    SemanticTraversabilityOcTree::tree_type_initializer_;

SemanticTraversabilityOcTree::SemanticTraversabilityOcTree(double resolution)
    : Base(resolution), phi_(-0.1f), psi_(1.0f),
      semantic_max_log_odds_(logodds(0.99)),
      semantic_min_log_odds_(logodds(0.0001)), rise_alpha_(0.70f),
      fall_alpha_(0.15f), merge_epsilon_(0.05f),
      confidence_merge_epsilon_(0.10f)
{
    tree_type_initializer_.ensureLinking();
}

SemanticTraversabilityOcTree* SemanticTraversabilityOcTree::create() const
{
    return new SemanticTraversabilityOcTree(resolution);
}

std::string SemanticTraversabilityOcTree::getTreeType() const
{
    return "SemanticTraversabilityOcTree";
}

void SemanticTraversabilityOcTree::setSemanticMaxProbability(float value)
{
    semantic_max_log_odds_ = logodds(clampUnit(value));
}

void SemanticTraversabilityOcTree::setSemanticMinProbability(float value)
{
    semantic_min_log_odds_ = logodds(clampUnit(value));
}

void SemanticTraversabilityOcTree::setTraversabilityRiseAlpha(float value)
{
    rise_alpha_ = clampUnit(value);
}

void SemanticTraversabilityOcTree::setTraversabilityFallAlpha(float value)
{
    fall_alpha_ = clampUnit(value);
}

void SemanticTraversabilityOcTree::setTraversabilityMergeEpsilon(float value)
{
    merge_epsilon_ = std::max(0.0f, value);
}

void SemanticTraversabilityOcTree::setConfidenceMergeEpsilon(float value)
{
    confidence_merge_epsilon_ = std::max(0.0f, value);
}

void SemanticTraversabilityOcTree::updateSemanticObservation(
    Node* node, const ColorOcTreeNode::Color& color_value)
{
    if (node == nullptr || color_value == ColorOcTreeNode::Color(255, 255, 255))
        return;
    node->semantics_ = node->isSemanticsSet()
        ? SemanticsLogOdds::fuseObs(node->semantics_, color_value, phi_, psi_,
                                    semantic_max_log_odds_, semantic_min_log_odds_)
        : SemanticsLogOdds::initSemantics(color_value, node->getLogOdds(), phi_, psi_,
                                         semantic_max_log_odds_, semantic_min_log_odds_);
}

void SemanticTraversabilityOcTree::updateTraversabilityObservation(
    Node* node, float measured, float confidence, uint32_t samples)
{
    if (node == nullptr || !std::isfinite(measured))
        return;
    measured = clampUnit(measured);
    confidence = clampUnit(confidence);
    TraversabilityState& state = node->traversability_;
    if (!state.valid)
    {
        state.measured_cost = measured;
        state.confidence = confidence;
        state.observations = samples;
        state.valid = true;
    }
    else
    {
        const float alpha = measured >= state.measured_cost ? rise_alpha_ : fall_alpha_;
        state.measured_cost += alpha * confidence * (measured - state.measured_cost);
        state.confidence = clampUnit(
            state.confidence + (1.0f - state.confidence) * confidence);
        const uint64_t count = static_cast<uint64_t>(state.observations) + samples;
        state.observations = static_cast<uint32_t>(
            std::min<uint64_t>(count, std::numeric_limits<uint32_t>::max()));
    }
    state.max_cost = state.measured_cost;
    state.mean_cost = state.measured_cost;
    state.valid_fraction = 1.0f;
}

SemanticTraversabilityOcTree::Node* SemanticTraversabilityOcTree::updateEndpoint(
    const OcTreeKey& key,
    const std::vector<SemanticObservation>& semantic_observations,
    const ColorOcTreeNode::Color& display_color,
    bool traversability_valid,
    float traversability,
    float traversability_confidence,
    uint32_t traversability_samples)
{
    Node* node = Base::updateNode(key, true, true);
    if (node == nullptr)
        return nullptr;

    if (node->isColorSet())
    {
        const ColorOcTreeNode::Color previous = node->getColor();
        node->setColor((static_cast<unsigned int>(previous.r) + display_color.r) / 2,
                       (static_cast<unsigned int>(previous.g) + display_color.g) / 2,
                       (static_cast<unsigned int>(previous.b) + display_color.b) / 2);
    }
    else
    {
        node->setColor(display_color);
    }

    for (const SemanticObservation& observation : semantic_observations)
    {
        for (uint32_t vote = 0; vote < observation.votes; ++vote)
            updateSemanticObservation(node, observation.color);
    }
    if (traversability_valid)
        updateTraversabilityObservation(node, traversability,
                                        traversability_confidence,
                                        traversability_samples);
    return node;
}

SemanticTraversabilityOcTree::Node* SemanticTraversabilityOcTree::updateFree(
    const OcTreeKey& key)
{
    // Occupancy-only update by design. A free-space ray is not a cost=0
    // surface observation and must not modify traversability or semantics.
    return Base::updateNode(key, false, true);
}

void SemanticTraversabilityOcTree::updateInnerOccupancyRecursive(
    Node* node, unsigned int depth)
{
    if (!nodeHasChildren(node))
        return;
    if (depth < tree_depth)
    {
        for (size_t index = 0; index < 8; ++index)
        {
            if (nodeChildExists(node, index))
                updateInnerOccupancyRecursive(getNodeChild(node, index), depth + 1);
        }
    }
    node->updateOccupancyChildren();
    node->updateColorChildren();
    node->updateSemanticsChildren();
    node->updateTraversabilityChildren();
}

void SemanticTraversabilityOcTree::updateInnerOccupancy()
{
    if (root != nullptr)
        updateInnerOccupancyRecursive(root, 0);
}

bool SemanticTraversabilityOcTree::isNodeCollapsible(const Node* node) const
{
    if (!nodeChildExists(node, 0))
        return false;
    const Node* first = getNodeChild(node, 0);
    if (nodeHasChildren(first))
        return false;

    float min_cost = first->traversability_.mean_cost;
    float max_cost = first->traversability_.max_cost;
    float min_confidence = first->traversability_.confidence;
    float max_confidence = first->traversability_.confidence;
    for (size_t index = 1; index < 8; ++index)
    {
        if (!nodeChildExists(node, index))
            return false;
        const Node* child = getNodeChild(node, index);
        if (nodeHasChildren(child) || child->getValue() != first->getValue() ||
            child->semantics_ != first->semantics_ ||
            child->traversability_.valid != first->traversability_.valid ||
            child->traversability_.valid_fraction !=
                first->traversability_.valid_fraction ||
            child->traversability_.observations !=
                first->traversability_.observations)
            return false;

        if (child->traversability_.valid)
        {
            min_cost = std::min(min_cost, child->traversability_.mean_cost);
            max_cost = std::max(max_cost, child->traversability_.max_cost);
            min_confidence = std::min(min_confidence, child->traversability_.confidence);
            max_confidence = std::max(max_confidence, child->traversability_.confidence);
        }
    }

    return !first->traversability_.valid ||
           (max_cost - min_cost <= merge_epsilon_ &&
            max_confidence - min_confidence <= confidence_merge_epsilon_);
}

bool SemanticTraversabilityOcTree::pruneNode(Node* node)
{
    if (!isNodeCollapsible(node))
        return false;

    node->copyData(*getNodeChild(node, 0));
    if (node->isColorSet())
        node->setColor(node->getAverageChildColor());
    node->updateTraversabilityChildren();

    for (size_t index = 0; index < 8; ++index)
        deleteNodeChild(node, index);
    delete[] node->children;
    node->children = nullptr;
    return true;
}

void SemanticTraversabilityOcTree::setUseSemanticColor(bool enabled)
{
    for (tree_iterator iterator = begin_tree(), finish = end_tree();
         iterator != finish; ++iterator)
        iterator->use_semantic_color_ = enabled;
}

void SemanticTraversabilityOcTree::setWriteExtendedData(bool enabled)
{
    for (tree_iterator iterator = begin_tree(), finish = end_tree();
         iterator != finish; ++iterator)
        iterator->write_extended_data_ = enabled;
}

bool SemanticTraversabilityOcTree::getLegacyRayRLE(
    const point3d& origin, const point3d& end, semantic_octomap::RayRLE& output)
{
    KeyRay* ray = &keyrays.at(0);
    if (!computeRayKeys(origin, end, *ray))
        return false;
    std::vector<OcTreeKey> keys(ray->begin(), ray->end());
    if (keys.empty())
    {
        OcTreeKey key;
        if (!coordToKeyChecked(origin, key))
            return false;
        keys.push_back(key);
    }

    for (const OcTreeKey& key : keys)
    {
        const std::array<double, 4> values = semanticLogOdds(search(key));
        if (!output.le_list.empty() && sameLegacyEntry(output.le_list.back(), values))
        {
            output.le_list.back().le[0] += 1.0;
            continue;
        }
        semantic_octomap::LE entry;
        entry.le.push_back(1.0);
        entry.le.insert(entry.le.end(), values.begin(), values.end());
        output.le_list.push_back(entry);
    }
    return true;
}

bool SemanticTraversabilityOcTree::getTraversabilityRayRLE(
    const point3d& origin, const point3d& end,
    semantic_octomap::TraversabilityRayRLE& output)
{
    KeyRay* ray = &keyrays.at(0);
    if (!computeRayKeys(origin, end, *ray))
        return false;
    std::vector<OcTreeKey> keys(ray->begin(), ray->end());
    if (keys.empty())
    {
        OcTreeKey key;
        if (!coordToKeyChecked(origin, key))
            return false;
        keys.push_back(key);
    }

    for (const OcTreeKey& key : keys)
    {
        semantic_octomap::TraversabilityLE entry =
            makeTraversabilityEntry(search(key));
        if (!output.entries.empty() &&
            sameTraversabilityEntry(output.entries.back(), entry))
        {
            ++output.entries.back().run_length;
        }
        else
        {
            output.entries.push_back(entry);
        }
    }
    return true;
}
}  // namespace octomap
