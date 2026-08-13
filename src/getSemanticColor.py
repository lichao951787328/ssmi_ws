#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np

from collections import defaultdict, Counter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


bridge = CvBridge()
counts = defaultdict(Counter)
frame_count = 0
channel_warning_printed = False

label_names = {
    0: "background/unlabeled",
    1: "chair",
    2: "television",
    3: "table",
    4: "boundary_wall",
    5: "maze_wall",
    6: "dynamic_obstacle",
    7: "staircase",
}


def callback(label_msg, color_msg):
    global frame_count
    global channel_warning_printed

    try:
        # 两张图统一转换成标准 RGB 顺序
        label_rgb = bridge.imgmsg_to_cv2(
            label_msg,
            desired_encoding="rgb8"
        )
        semantic_rgb = bridge.imgmsg_to_cv2(
            color_msg,
            desired_encoding="rgb8"
        )
    except Exception as exc:
        rospy.logerr("cv_bridge conversion failed: %s", exc)
        return

    if label_rgb.shape != semantic_rgb.shape:
        rospy.logwarn(
            "Image size mismatch: labels=%s, colors=%s",
            label_rgb.shape,
            semantic_rgb.shape
        )
        return

    # Gazebo Fortress 将 label 重复写入三个通道：
    # label 1 -> RGB(1,1,1)
    # label 2 -> RGB(2,2,2)
    # 因此直接读取第一个通道即可。
    label_r = label_rgb[:, :, 0]
    label_g = label_rgb[:, :, 1]
    label_b = label_rgb[:, :, 2]

    # 检查三个 label 通道是否一致
    if (
        not np.array_equal(label_r, label_g)
        or not np.array_equal(label_r, label_b)
    ):
        if not channel_warning_printed:
            rospy.logwarn(
                "The three channels in labels_map are not identical. "
                "The script will continue using the R channel."
            )
            channel_warning_printed = True

    labels = label_r.astype(np.uint32)

    labels_flat = labels.reshape(-1)
    colors_flat = semantic_rgb.reshape(-1, 3)

    # 分别统计每个 label 所对应的 RGB
    for label in np.unique(labels_flat):
        mask = labels_flat == label

        colors, occurrences = np.unique(
            colors_flat[mask],
            axis=0,
            return_counts=True
        )

        for color, number in zip(colors, occurrences):
            rgb = tuple(int(value) for value in color)
            counts[int(label)][rgb] += int(number)

    frame_count += 1

    if frame_count % 20 == 0:
        rospy.loginfo(
            "Processed %d exactly synchronized frames",
            frame_count
        )


def print_result():
    print()
    print("============== label -> semantic -> RGB ==============")
    print("Processed exactly synchronized frames:", frame_count)

    if frame_count == 0:
        print("No synchronized image pairs were received.")
        print("If both topics are active, try the approximate-time version.")
        print("======================================================")
        return

    for label in sorted(counts):
        total = sum(counts[label].values())
        most_common_rgb, most_common_count = counts[label].most_common(1)[0]
        purity = 100.0 * most_common_count / total
        semantic_name = label_names.get(label, "unknown")

        print(
            "label {:3d} | {:20s} | "
            "RGB({:3d}, {:3d}, {:3d}) | "
            "pixels={} | purity={:.4f}%".format(
                label,
                semantic_name,
                most_common_rgb[0],
                most_common_rgb[1],
                most_common_rgb[2],
                total,
                purity
            )
        )

        if len(counts[label]) > 1:
            print("          top colors:", counts[label].most_common(5))

    print("======================================================")


rospy.init_node(
    "semantic_label_rgb_counter",
    anonymous=True
)
rospy.on_shutdown(print_result)

label_sub = message_filters.Subscriber(
    "/semantic/labels_map",
    Image,
    queue_size=10
)

color_sub = message_filters.Subscriber(
    "/semantic/colored_map",
    Image,
    queue_size=10
)

# labels_map 和 colored_map 来自同一个 Gazebo 分割相机，
# 正常情况下应当具有相同时间戳。
sync = message_filters.TimeSynchronizer(
    [label_sub, color_sub],
    queue_size=10
)
sync.registerCallback(callback)

print("正在严格同步统计 label -> semantic RGB。")
print("请让相机看到需要统计的物体。")
print("统计完成后按 Ctrl+C 输出结果。")

rospy.spin()
