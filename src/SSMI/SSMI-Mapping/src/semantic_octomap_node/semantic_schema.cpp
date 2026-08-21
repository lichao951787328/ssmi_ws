#include <semantic_octomap_node/semantic_schema.h>

#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace semantic_octomap
{
namespace
{

const XmlRpc::XmlRpcValue& requireMember(const XmlRpc::XmlRpcValue& value,
                                         const std::string& name,
                                         const std::string& context)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !value.hasMember(name))
        throw std::runtime_error(context + " requires '" + name + "'");
    return value[name];
}

std::string readString(const XmlRpc::XmlRpcValue& value,
                       const std::string& context)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeString)
        throw std::runtime_error(context + " must be a string");
    return static_cast<std::string>(value);
}

int readInt(const XmlRpc::XmlRpcValue& value, const std::string& context)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt)
        throw std::runtime_error(context + " must be an integer");
    return static_cast<int>(value);
}

double readNumber(const XmlRpc::XmlRpcValue& value,
                  const std::string& context)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        return static_cast<double>(value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
        return static_cast<int>(value);
    throw std::runtime_error(context + " must be numeric");
}

bool readBool(const XmlRpc::XmlRpcValue& value, const std::string& context)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        throw std::runtime_error(context + " must be true or false");
    return static_cast<bool>(value);
}

SemanticRgb readRgb(const XmlRpc::XmlRpcValue& value,
                    const std::string& context)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
        throw std::runtime_error(context + " must be an RGB array with three integers");
    SemanticRgb rgb;
    for (int channel = 0; channel < 3; ++channel)
    {
        const int component = readInt(value[channel], context);
        if (component < 0 || component > 255)
            throw std::runtime_error(context + " RGB values must be in [0, 255]");
        rgb[static_cast<std::size_t>(channel)] = static_cast<uint8_t>(component);
    }
    return rgb;
}

SemanticRole readRole(const XmlRpc::XmlRpcValue& value,
                      const std::string& context)
{
    const std::string role = readString(value, context);
    if (role == "terrain")
        return SemanticRole::Terrain;
    if (role == "static_obstacle")
        return SemanticRole::StaticObstacle;
    if (role == "dynamic_obstacle")
        return SemanticRole::DynamicObstacle;
    if (role == "ignore")
        return SemanticRole::Ignore;
    throw std::runtime_error(
        context + " must be terrain, static_obstacle, dynamic_obstacle, or ignore");
}

}  // namespace

SemanticSchema SemanticSchema::fromRos(const ros::NodeHandle& nh,
                                       const std::string& parameter)
{
    XmlRpc::XmlRpcValue root;
    if (!nh.getParam(parameter, root))
        throw std::runtime_error("required semantic schema parameter '" +
                                 parameter + "' is not loaded");
    return fromXmlRpc(root);
}

SemanticSchema SemanticSchema::fromXmlRpc(const XmlRpc::XmlRpcValue& root)
{
    if (root.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        throw std::runtime_error("semantic_schema must be a YAML mapping");

    SemanticSchema schema;
    const int version = readInt(requireMember(root, "version", "semantic_schema"),
                                "semantic_schema/version");
    if (version != 1)
        throw std::runtime_error("semantic_schema/version must currently be 1");

    if (root.hasMember("input"))
    {
        const XmlRpc::XmlRpcValue& input = root["input"];
        if (input.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            throw std::runtime_error("semantic_schema/input must be a mapping");
        if (input.hasMember("label_field"))
            schema.label_field_ = readString(input["label_field"],
                                             "semantic_schema/input/label_field");
        if (input.hasMember("traversability_field"))
            schema.traversability_field_ = readString(
                input["traversability_field"],
                "semantic_schema/input/traversability_field");
    }
    if (schema.label_field_.empty() || schema.traversability_field_.empty())
        throw std::runtime_error("semantic_schema input field names must not be empty");

    const XmlRpc::XmlRpcValue& defaults =
        requireMember(root, "defaults", "semantic_schema");
    const int initial_floor_label = readInt(
        requireMember(defaults, "initial_floor_label", "semantic_schema/defaults"),
        "semantic_schema/defaults/initial_floor_label");
    if (initial_floor_label < 0)
        throw std::runtime_error("initial_floor_label must be non-negative");
    schema.initial_floor_label_ = static_cast<uint32_t>(initial_floor_label);

    const XmlRpc::XmlRpcValue& unknown =
        requireMember(root, "unknown", "semantic_schema");
    const std::string unknown_policy = readString(
        requireMember(unknown, "policy", "semantic_schema/unknown"),
        "semantic_schema/unknown/policy");
    if (unknown_policy == "exclude")
        schema.unknown_policy_ = UnknownPolicy::Exclude;
    else if (unknown_policy == "map_to_fallback")
        schema.unknown_policy_ = UnknownPolicy::MapToFallback;
    else
        throw std::runtime_error(
            "semantic_schema/unknown/policy must be exclude or map_to_fallback");
    if (unknown.hasMember("name"))
        schema.unknown_name_ = readString(unknown["name"],
                                          "semantic_schema/unknown/name");
    if (unknown.hasMember("meaning"))
        schema.unknown_meaning_ = readString(unknown["meaning"],
                                             "semantic_schema/unknown/meaning");
    if (unknown.hasMember("rgb"))
        schema.unknown_rgb_ = readRgb(unknown["rgb"],
                                      "semantic_schema/unknown/rgb");
    if (unknown.hasMember("fallback_label"))
    {
        const int fallback = readInt(unknown["fallback_label"],
                                     "semantic_schema/unknown/fallback_label");
        if (fallback < 0)
            throw std::runtime_error("unknown fallback_label must be non-negative");
        schema.unknown_fallback_label_ = static_cast<uint32_t>(fallback);
    }

    const XmlRpc::XmlRpcValue& traversability =
        requireMember(root, "traversability", "semantic_schema");
    schema.traversability_override_ = readBool(
        requireMember(traversability, "override_semantics",
                      "semantic_schema/traversability"),
        "semantic_schema/traversability/override_semantics");
    const double threshold = readNumber(
        requireMember(traversability, "obstacle_threshold",
                      "semantic_schema/traversability"),
        "semantic_schema/traversability/obstacle_threshold");
    if (threshold < 0.0 || threshold > 1.0)
        throw std::runtime_error("traversability obstacle_threshold must be in [0, 1]");
    schema.obstacle_threshold_ = static_cast<float>(threshold);
    const int obstacle_label = readInt(
        requireMember(traversability, "obstacle_label",
                      "semantic_schema/traversability"),
        "semantic_schema/traversability/obstacle_label");
    if (obstacle_label < 0)
        throw std::runtime_error("traversability obstacle_label must be non-negative");
    schema.traversability_obstacle_label_ =
        static_cast<uint32_t>(obstacle_label);

    const XmlRpc::XmlRpcValue& classes =
        requireMember(root, "classes", "semantic_schema");
    if (classes.getType() != XmlRpc::XmlRpcValue::TypeArray || classes.size() == 0)
        throw std::runtime_error("semantic_schema/classes must be a non-empty list");
    schema.classes_.reserve(static_cast<std::size_t>(classes.size()));
    for (int index = 0; index < classes.size(); ++index)
    {
        const XmlRpc::XmlRpcValue& item = classes[index];
        const std::string context = "semantic_schema/classes[" +
            std::to_string(index) + "]";
        const int label = readInt(requireMember(item, "label", context),
                                  context + "/label");
        if (label < 0)
            throw std::runtime_error(context + "/label must be non-negative");

        SemanticClass semantic_class;
        semantic_class.label = static_cast<uint32_t>(label);
        semantic_class.name = readString(requireMember(item, "name", context),
                                         context + "/name");
        if (semantic_class.name.empty())
            throw std::runtime_error(context + "/name must not be empty");
        if (item.hasMember("meaning"))
            semantic_class.meaning = readString(item["meaning"],
                                                context + "/meaning");
        semantic_class.rgb = readRgb(requireMember(item, "rgb", context),
                                     context + "/rgb");
        semantic_class.role = readRole(requireMember(item, "role", context),
                                       context + "/role");
        semantic_class.admit_to_global_map = readBool(
            requireMember(item, "global_map", context), context + "/global_map");
        if (semantic_class.admit_to_global_map &&
            (semantic_class.role == SemanticRole::DynamicObstacle ||
             semantic_class.role == SemanticRole::Ignore))
        {
            throw std::runtime_error(
                context + " cannot set global_map=true for role " +
                roleName(semantic_class.role));
        }

        if (item.hasMember("input_rgb_aliases"))
        {
            const XmlRpc::XmlRpcValue& aliases = item["input_rgb_aliases"];
            if (aliases.getType() != XmlRpc::XmlRpcValue::TypeArray)
                throw std::runtime_error(context + "/input_rgb_aliases must be a list");
            semantic_class.input_rgb_aliases.reserve(
                static_cast<std::size_t>(aliases.size()));
            for (int alias = 0; alias < aliases.size(); ++alias)
                semantic_class.input_rgb_aliases.push_back(
                    readRgb(aliases[alias], context + "/input_rgb_aliases"));
        }

        const std::size_t class_index = schema.classes_.size();
        if (!schema.label_to_index_.emplace(semantic_class.label, class_index).second)
            throw std::runtime_error("duplicate semantic label " +
                                     std::to_string(semantic_class.label));
        const auto add_color = [&](const SemanticRgb& rgb, const char* kind) {
            const uint32_t packed = packRgb(rgb);
            const auto existing = schema.color_to_index_.find(packed);
            if (existing != schema.color_to_index_.end() &&
                existing->second != class_index)
            {
                std::ostringstream message;
                message << "duplicate " << kind << " RGB ["
                        << static_cast<int>(rgb[0]) << ", "
                        << static_cast<int>(rgb[1]) << ", "
                        << static_cast<int>(rgb[2]) << "]";
                throw std::runtime_error(message.str());
            }
            schema.color_to_index_[packed] = class_index;
        };
        add_color(semantic_class.rgb, "class");
        for (const SemanticRgb& alias : semantic_class.input_rgb_aliases)
            add_color(alias, "alias");
        schema.classes_.push_back(std::move(semantic_class));
    }

    if (schema.label_to_index_.count(schema.traversability_obstacle_label_) == 0u)
        throw std::runtime_error("traversability obstacle_label is not defined in classes");
    const SemanticClass& obstacle =
        schema.classForLabel(schema.traversability_obstacle_label_);
    if (obstacle.role != SemanticRole::StaticObstacle ||
        !obstacle.admit_to_global_map)
        throw std::runtime_error(
            "traversability obstacle_label must be an admitted static_obstacle");

    if (schema.label_to_index_.count(schema.initial_floor_label_) == 0u)
        throw std::runtime_error("initial_floor_label is not defined in classes");
    const SemanticClass& initial_floor = schema.initialFloorClass();
    if (initial_floor.role != SemanticRole::Terrain ||
        !initial_floor.admit_to_global_map)
        throw std::runtime_error(
            "initial_floor_label must be an admitted terrain class");

    if (schema.unknown_policy_ == UnknownPolicy::MapToFallback)
    {
        if (schema.label_to_index_.count(schema.unknown_fallback_label_) == 0u)
            throw std::runtime_error("unknown fallback_label is not defined in classes");
        if (!schema.classForLabel(schema.unknown_fallback_label_).admit_to_global_map)
            throw std::runtime_error("unknown fallback_label must be admitted to global_map");
    }

    schema.loaded_ = true;
    return schema;
}

ResolvedSemantic SemanticSchema::resolvedClass(const std::size_t index) const
{
    const SemanticClass& semantic_class = classes_.at(index);
    ResolvedSemantic resolved;
    resolved.known = true;
    resolved.admitted = semantic_class.admit_to_global_map;
    resolved.label = semantic_class.label;
    resolved.rgb = semantic_class.rgb;
    resolved.role = semantic_class.role;
    return resolved;
}

ResolvedSemantic SemanticSchema::resolveLabel(const uint32_t label,
                                               const bool high_cost_obstacle) const
{
    const auto found = label_to_index_.find(label);
    if (found != label_to_index_.end())
    {
        ResolvedSemantic resolved = resolvedClass(found->second);
        // Excluded classes (notably dynamics) can never be promoted to a static
        // wall merely because their local traversability cost is high.
        if (resolved.admitted && traversability_override_ && high_cost_obstacle)
        {
            return resolvedClass(label_to_index_.at(
                traversability_obstacle_label_));
        }
        return resolved;
    }

    if (unknown_policy_ == UnknownPolicy::MapToFallback)
        return resolvedClass(label_to_index_.at(unknown_fallback_label_));

    ResolvedSemantic unknown;
    unknown.rgb = unknown_rgb_;
    return unknown;
}

ResolvedSemantic SemanticSchema::resolveColor(const uint32_t packed_rgb) const
{
    const auto found = color_to_index_.find(packed_rgb & 0x00ffffffu);
    if (found != color_to_index_.end())
        return resolvedClass(found->second);
    if (unknown_policy_ == UnknownPolicy::MapToFallback)
        return resolvedClass(label_to_index_.at(unknown_fallback_label_));
    ResolvedSemantic unknown;
    unknown.rgb = unknown_rgb_;
    return unknown;
}

const SemanticClass& SemanticSchema::classForLabel(const uint32_t label) const
{
    const auto found = label_to_index_.find(label);
    if (found == label_to_index_.end())
        throw std::out_of_range("semantic label is not defined: " +
                                std::to_string(label));
    return classes_.at(found->second);
}

const SemanticClass& SemanticSchema::traversabilityObstacleClass() const
{
    return classForLabel(traversability_obstacle_label_);
}

const SemanticClass& SemanticSchema::initialFloorClass() const
{
    return classForLabel(initial_floor_label_);
}

uint32_t SemanticSchema::packRgb(const SemanticRgb& rgb)
{
    return (static_cast<uint32_t>(rgb[0]) << 16) |
           (static_cast<uint32_t>(rgb[1]) << 8) |
           static_cast<uint32_t>(rgb[2]);
}

const char* SemanticSchema::roleName(const SemanticRole role)
{
    switch (role)
    {
        case SemanticRole::Terrain: return "terrain";
        case SemanticRole::StaticObstacle: return "static_obstacle";
        case SemanticRole::DynamicObstacle: return "dynamic_obstacle";
        case SemanticRole::Ignore: return "ignore";
    }
    return "ignore";
}

}  // namespace semantic_octomap
