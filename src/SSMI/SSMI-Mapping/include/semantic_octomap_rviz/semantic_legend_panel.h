#ifndef SEMANTIC_OCTOMAP_RVIZ_SEMANTIC_LEGEND_PANEL_H
#define SEMANTIC_OCTOMAP_RVIZ_SEMANTIC_LEGEND_PANEL_H

#include <rviz/panel.h>

namespace semantic_octomap
{

class SemanticLegendPanel : public rviz::Panel
{
    Q_OBJECT

public:
    explicit SemanticLegendPanel(QWidget* parent = nullptr);
};

}  // namespace semantic_octomap

#endif  // SEMANTIC_OCTOMAP_RVIZ_SEMANTIC_LEGEND_PANEL_H
