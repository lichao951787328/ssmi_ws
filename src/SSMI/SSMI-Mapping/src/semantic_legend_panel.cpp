#include <semantic_octomap_rviz/semantic_legend_panel.h>

#include <pluginlib/class_list_macros.h>

#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include <array>
#include <vector>

namespace semantic_octomap
{
namespace
{
using Rgb = std::array<int, 3>;

struct LegendEntry
{
    Rgb color;
    const char* label;
};

const std::vector<LegendEntry> kLegendEntries = {
    {{{128, 64, 128}}, "0  道路 (road)"},
    {{{244, 35, 232}}, "1  人行道 (sidewalk)"},
    {{{70, 70, 70}}, "2  建筑 (building)"},
    {{{102, 102, 156}}, "3  墙 / 高代价回退 (wall / high-cost)"},
    {{{190, 153, 153}}, "4  栅栏 (fence)"},
    {{{153, 153, 153}}, "5  杆 (pole)"},
    {{{250, 170, 30}}, "6  交通灯 (traffic light)"},
    {{{220, 220, 0}}, "7  交通标志 (traffic sign)"},
    {{{107, 142, 35}}, "8  植被 (vegetation)"},
    {{{152, 251, 152}}, "9  地形 (terrain)"},
    {{{70, 130, 180}}, "10 天空 (sky)"},
    {{{255, 0, 255}}, "11–18 动态障碍合并 (person / vehicle)"},
    {{{255, 255, 255}}, "无效或未知标签 (unknown)"},
};

QLabel* makeColorSwatch(const Rgb& rgb)
{
    QLabel* swatch = new QLabel;
    swatch->setFixedSize(24, 18);
    swatch->setFrameShape(QFrame::Box);
    swatch->setStyleSheet(
        QString("background-color: rgb(%1, %2, %3); border: 1px solid #303030;")
            .arg(rgb[0]).arg(rgb[1]).arg(rgb[2]));
    return swatch;
}
}  // namespace

SemanticLegendPanel::SemanticLegendPanel(QWidget* parent)
    : rviz::Panel(parent)
{
    setMinimumWidth(320);

    QWidget* legend_widget = new QWidget;
    QGridLayout* legend_layout = new QGridLayout(legend_widget);
    legend_layout->setContentsMargins(8, 8, 8, 8);
    legend_layout->setHorizontalSpacing(10);
    legend_layout->setVerticalSpacing(5);

    QLabel* title = new QLabel("SSMI 语义颜色图例");
    QFont title_font = title->font();
    title_font.setBold(true);
    title_font.setPointSize(title_font.pointSize() + 2);
    title->setFont(title_font);
    legend_layout->addWidget(title, 0, 0, 1, 2);

    int row = 1;
    for (const LegendEntry& entry : kLegendEntries)
    {
        legend_layout->addWidget(makeColorSwatch(entry.color), row, 0,
                                 Qt::AlignLeft | Qt::AlignVCenter);
        QLabel* label = new QLabel(QString::fromUtf8(entry.label));
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        legend_layout->addWidget(label, row, 1);
        ++row;
    }

    QLabel* note = new QLabel(
        "说明：适配器把原始标签 11–18 统一映射为洋红色；"
        "可通行/未知点若 traversability ≥ 0.6，则按墙的颜色显示。");
    note->setWordWrap(true);
    note->setStyleSheet("color: #555555; margin-top: 8px;");
    legend_layout->addWidget(note, row, 0, 1, 2);
    legend_layout->setColumnStretch(1, 1);
    legend_layout->setRowStretch(row + 1, 1);

    QScrollArea* scroll_area = new QScrollArea;
    scroll_area->setWidgetResizable(true);
    scroll_area->setFrameShape(QFrame::NoFrame);
    scroll_area->setWidget(legend_widget);

    QVBoxLayout* panel_layout = new QVBoxLayout(this);
    panel_layout->setContentsMargins(0, 0, 0, 0);
    panel_layout->addWidget(scroll_area);
}

}  // namespace semantic_octomap

PLUGINLIB_EXPORT_CLASS(semantic_octomap::SemanticLegendPanel, rviz::Panel)
