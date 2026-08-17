#ifndef SEMANTIC_OCTOMAP_GRID_SEMANTIC_ADMISSION_H
#define SEMANTIC_OCTOMAP_GRID_SEMANTIC_ADMISSION_H

#include <array>
#include <cstdint>

namespace semantic_octomap
{

using SemanticRgb = std::array<uint8_t, 3>;

enum class GlobalAdmissionClass
{
    Unexpected,
    Terrain,
    StaticObstacle
};

// Cityscapes colors are kept because they are the on-wire semantic encoding
// consumed by the existing SemanticOcTree implementation.
inline const std::array<SemanticRgb, 19>& cityscapesPalette()
{
    static const std::array<SemanticRgb, 19> palette = {{
        {{128, 64, 128}}, {{244, 35, 232}}, {{70, 70, 70}},
        {{102, 102, 156}}, {{190, 153, 153}}, {{153, 153, 153}},
        {{250, 170, 30}}, {{220, 220, 0}}, {{107, 142, 35}},
        {{152, 251, 152}}, {{70, 130, 180}}, {{220, 20, 60}},
        {{255, 0, 0}}, {{0, 0, 142}}, {{0, 0, 70}},
        {{0, 60, 100}}, {{0, 80, 100}}, {{0, 0, 230}},
        {{119, 11, 32}}
    }};
    return palette;
}

inline GlobalAdmissionClass classifyGlobalAdmissionLabel(uint32_t label)
{
    if (label == 0 || label == 1 || label == 9)
        return GlobalAdmissionClass::Terrain;
    if (label >= 2 && label <= 8)
        return GlobalAdmissionClass::StaticObstacle;
    return GlobalAdmissionClass::Unexpected;
}

inline bool admittedSemanticColor(uint32_t label, SemanticRgb& color)
{
    if (classifyGlobalAdmissionLabel(label) == GlobalAdmissionClass::Unexpected)
        return false;
    color = cityscapesPalette()[label];
    return true;
}

}  // namespace semantic_octomap

#endif  // SEMANTIC_OCTOMAP_GRID_SEMANTIC_ADMISSION_H
