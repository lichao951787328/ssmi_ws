#ifndef SEMANTIC_OCTOMAP_SEMANTIC_SCHEMA_H
#define SEMANTIC_OCTOMAP_SEMANTIC_SCHEMA_H

#include <ros/node_handle.h>
#include <XmlRpcValue.h>

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace semantic_octomap
{

using SemanticRgb = std::array<uint8_t, 3>;

enum class SemanticRole
{
    Terrain,
    StaticObstacle,
    DynamicObstacle,
    Ignore
};

struct SemanticClass
{
    uint32_t label = 0;
    std::string name;
    std::string meaning;
    SemanticRgb rgb{{0, 0, 0}};
    std::vector<SemanticRgb> input_rgb_aliases;
    SemanticRole role = SemanticRole::Ignore;
    bool admit_to_global_map = false;
};

struct ResolvedSemantic
{
    bool known = false;
    bool admitted = false;
    uint32_t label = 0;
    SemanticRgb rgb{{0, 0, 0}};
    SemanticRole role = SemanticRole::Ignore;
};

// Runtime semantic contract shared by the input adapter, OctoMap insertion
// policy, semantic sensor compatibility path, and RViz legend. The YAML is
// loaded into /semantic_schema by the launch files.
class SemanticSchema
{
public:
    enum class UnknownPolicy
    {
        Exclude,
        MapToFallback
    };

    SemanticSchema() = default;

    static SemanticSchema fromRos(const ros::NodeHandle& nh,
                                  const std::string& parameter = "/semantic_schema");
    static SemanticSchema fromXmlRpc(const XmlRpc::XmlRpcValue& root);

    ResolvedSemantic resolveLabel(uint32_t label,
                                  bool high_cost_obstacle = false) const;
    ResolvedSemantic resolveColor(uint32_t packed_rgb) const;

    const std::vector<SemanticClass>& classes() const { return classes_; }
    const SemanticClass& classForLabel(uint32_t label) const;
    const SemanticClass& traversabilityObstacleClass() const;
    const SemanticClass& initialFloorClass() const;

    const std::string& labelField() const { return label_field_; }
    const std::string& traversabilityField() const { return traversability_field_; }
    float obstacleThreshold() const { return obstacle_threshold_; }
    bool traversabilityOverrideEnabled() const { return traversability_override_; }
    UnknownPolicy unknownPolicy() const { return unknown_policy_; }
    const std::string& unknownName() const { return unknown_name_; }
    const std::string& unknownMeaning() const { return unknown_meaning_; }
    const SemanticRgb& unknownRgb() const { return unknown_rgb_; }
    uint32_t unknownFallbackLabel() const { return unknown_fallback_label_; }

    bool isLoaded() const { return loaded_; }

    static uint32_t packRgb(const SemanticRgb& rgb);
    static const char* roleName(SemanticRole role);

private:
    ResolvedSemantic resolvedClass(std::size_t index) const;

    bool loaded_ = false;
    std::string label_field_ = "semantic_lable";
    std::string traversability_field_ = "traversability";
    std::vector<SemanticClass> classes_;
    std::unordered_map<uint32_t, std::size_t> label_to_index_;
    std::unordered_map<uint32_t, std::size_t> color_to_index_;

    UnknownPolicy unknown_policy_ = UnknownPolicy::Exclude;
    std::string unknown_name_ = "unknown";
    std::string unknown_meaning_ = "invalid or unconfigured semantic value";
    SemanticRgb unknown_rgb_{{127, 127, 127}};
    uint32_t unknown_fallback_label_ = 0;

    bool traversability_override_ = true;
    float obstacle_threshold_ = 0.75f;
    uint32_t traversability_obstacle_label_ = 3;
    uint32_t initial_floor_label_ = 0;
};

}  // namespace semantic_octomap

#endif  // SEMANTIC_OCTOMAP_SEMANTIC_SCHEMA_H
