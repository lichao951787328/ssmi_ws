#include <semantic_octomap_rviz/semantic_legend_panel.h>
#include <semantic_octomap_node/semantic_schema.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include <exception>

namespace semantic_octomap
{
namespace
{
QString roleText(const SemanticRole role)
{
    switch (role)
    {
        case SemanticRole::Terrain: return QString::fromUtf8("可通行地形");
        case SemanticRole::StaticObstacle: return QString::fromUtf8("静态障碍");
        case SemanticRole::DynamicObstacle: return QString::fromUtf8("动态障碍");
        case SemanticRole::Ignore: return QString::fromUtf8("忽略");
    }
    return QString::fromUtf8("忽略");
}

QLabel* makeColorSwatch(const SemanticRgb& rgb)
{
    QLabel* swatch = new QLabel;
    swatch->setFixedSize(24, 18);
    swatch->setFrameShape(QFrame::Box);
    swatch->setStyleSheet(
        QString("background-color: rgb(%1, %2, %3); border: 1px solid #303030;")
            .arg(static_cast<int>(rgb[0]))
            .arg(static_cast<int>(rgb[1]))
            .arg(static_cast<int>(rgb[2])));
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
    try
    {
        ros::NodeHandle nh;
        const SemanticSchema schema = SemanticSchema::fromRos(nh);
        for (const SemanticClass& entry : schema.classes())
        {
            legend_layout->addWidget(makeColorSwatch(entry.rgb), row, 0,
                                     Qt::AlignLeft | Qt::AlignVCenter);
            QString text = QString("%1  %2 — %3 [%4, %5]")
                .arg(entry.label)
                .arg(QString::fromStdString(entry.name))
                .arg(QString::fromStdString(entry.meaning))
                .arg(roleText(entry.role))
                .arg(entry.admit_to_global_map ?
                    QString::fromUtf8("进入全局图") :
                    QString::fromUtf8("不进入全局图"));
            if (!entry.input_rgb_aliases.empty())
            {
                text += QString::fromUtf8("；输入颜色别名：");
                for (const SemanticRgb& alias : entry.input_rgb_aliases)
                {
                    text += QString(" [%1,%2,%3]")
                        .arg(static_cast<int>(alias[0]))
                        .arg(static_cast<int>(alias[1]))
                        .arg(static_cast<int>(alias[2]));
                }
            }
            QLabel* label = new QLabel(text);
            label->setWordWrap(true);
            label->setTextInteractionFlags(Qt::TextSelectableByMouse);
            legend_layout->addWidget(label, row, 1);
            ++row;
        }

        legend_layout->addWidget(makeColorSwatch(schema.unknownRgb()), row, 0,
                                 Qt::AlignLeft | Qt::AlignVCenter);
        const bool exclude_unknown = schema.unknownPolicy() ==
            SemanticSchema::UnknownPolicy::Exclude;
        QLabel* unknown = new QLabel(
            QString::fromUtf8("未知类别 — %1 [%2]")
                .arg(QString::fromStdString(schema.unknownMeaning()))
                .arg(exclude_unknown ? QString::fromUtf8("排除") :
                                       QString::fromUtf8("映射到回退类别")));
        unknown->setWordWrap(true);
        unknown->setTextInteractionFlags(Qt::TextSelectableByMouse);
        legend_layout->addWidget(unknown, row, 1);
        ++row;

        QLabel* note = new QLabel(
            QString::fromUtf8(
                "说明：该图例直接读取 /semantic_schema。动态类别由全局准入开关排除；"
                "已准入类别的 traversability ≥ %1 时按标签 %2（%3）编码。")
                .arg(schema.obstacleThreshold(), 0, 'f', 2)
                .arg(schema.traversabilityObstacleClass().label)
                .arg(QString::fromStdString(
                    schema.traversabilityObstacleClass().name)));
        note->setWordWrap(true);
        note->setStyleSheet("color: #555555; margin-top: 8px;");
        legend_layout->addWidget(note, row, 0, 1, 2);
        ++row;
    }
    catch (const std::exception& error)
    {
        ROS_ERROR("Cannot load /semantic_schema for RViz legend: %s", error.what());
        QLabel* failure = new QLabel(
            QString::fromUtf8("无法加载 /semantic_schema：") +
            QString::fromUtf8(error.what()));
        failure->setWordWrap(true);
        failure->setStyleSheet("color: #b00020;");
        legend_layout->addWidget(failure, row, 0, 1, 2);
        ++row;
    }
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
