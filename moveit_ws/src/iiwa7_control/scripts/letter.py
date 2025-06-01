#!/usr/bin/env python3
# coding: utf-8

import freetype
import rospy
import copy
from geometry_msgs.msg import Pose, Quaternion

# 字符轮廓分解器类
class OutlineDecomposer:
    """
    将字体文件中的字符轮廓分解为离散的2D坐标点。
    """
    def __init__(self, num_samples_per_curve_segment):
        """
        参数:
            num_samples_per_curve_segment (int): 每个曲线段的采样点数量，最小值为2
        """
        self.points_2d_font_units = [] # 字体单位中的(x,y)元组列表
        self._current_pen_pos = None   # 当前画笔位置
        self._num_samples = max(2, num_samples_per_curve_segment) # 确保至少有2个采样点
    # 添加坐标点到路径列表：判断新点是否离上一个点足够远
    def _add_point(self, p_tuple):
        epsilon = 1e-5
        if not self.points_2d_font_units or \
           abs(self.points_2d_font_units[-1][0] - p_tuple[0]) > epsilon or \
           abs(self.points_2d_font_units[-1][1] - p_tuple[1]) > epsilon:
            self.points_2d_font_units.append(p_tuple)
    # 移动画笔
    def move_to(self, a, _=None):
        self._current_pen_pos = (a.x, a.y)
        self._add_point(self._current_pen_pos) 
        return 0
    # 直线：直线段
    def line_to(self, a, _=None):
        if self._current_pen_pos is None:
            rospy.logwarn("line_to called without prior move_to establishing current position.")
            self._current_pen_pos = (a.x, a.y)
            self._add_point(self._current_pen_pos)
            return 0
        
        self._current_pen_pos = (a.x, a.y)
        self._add_point(self._current_pen_pos)
        return 0
    # 二次贝塞尔曲线：简单曲线段
    def conic_to(self, control, to, _=None):
        if self._current_pen_pos is None:
            rospy.logwarn("conic_to called without prior move_to establishing current position.")
            return -1 
        # 三个控制点
        p0 = self._current_pen_pos       # 起点
        p1_ctrl = (control.x, control.y) # 控制点
        p2_end = (to.x, to.y)            # 终点
        # 参数化离散采样
        for i in range(1, self._num_samples):
            t = float(i) / (self._num_samples - 1) 
            omt = 1.0 - t
            x = omt**2 * p0[0] + 2*omt*t * p1_ctrl[0] + t**2 * p2_end[0]
            y = omt**2 * p0[1] + 2*omt*t * p1_ctrl[1] + t**2 * p2_end[1]
            self._add_point((x, y))
        self._current_pen_pos = p2_end 
        return 0
    # 三次贝塞尔曲线：复杂曲线段
    def cubic_to(self, control1, control2, to, _=None):
        if self._current_pen_pos is None:
            rospy.logwarn("cubic_to called without prior move_to establishing current position.")
            return -1
        # 四个控制点
        p0 = self._current_pen_pos         # 起点
        p1_ctrl = (control1.x, control1.y) # 第一个控制点
        p2_ctrl = (control2.x, control2.y) # 第二个控制点
        p3_end = (to.x, to.y)              # 终点
        # 参数化离散采样
        for i in range(1, self._num_samples):
            t = float(i) / (self._num_samples - 1)
            omt = 1.0 - t 
            x = omt**3 * p0[0] + \
                3*omt**2*t * p1_ctrl[0] + \
                3*omt*t**2 * p2_ctrl[0] + \
                t**3 * p3_end[0]
            y = omt**3 * p0[1] + \
                3*omt**2*t * p1_ctrl[1] + \
                3*omt*t**2 * p2_ctrl[1] + \
                t**3 * p3_end[1]
            self._add_point((x, y))
        self._current_pen_pos = p3_end 
        return 0
# 路径生成函数
def generate_letter_waypoints_from_font(char_to_draw, font_path, desired_letter_height_m, num_samples_per_curve, center_pose, rospy_instance=None):
    """
    坐标系转换：
    - 字体2D坐标（X轴）→ 机器人YZ平面的Y轴
    - 字体2D坐标（Y轴）→ 机器人YZ平面的Z轴
    - 机器人X轴位置保持固定（由center_pose指定）

    参数:
        char_to_draw (str): 要绘制的单个字符
        font_path (str): 字体文件绝对路径（.ttf或.otf格式）
        desired_letter_height_m (float): 目标高度，单位为米
        num_samples_per_curve (int): 每个贝塞尔曲线段的采样点数量
        center_pose (geometry_msgs.msg.Pose): 中心位姿
    """
    # 日志
    log_info = rospy_instance.loginfo if rospy_instance else rospy.loginfo
    log_err = rospy_instance.logerr if rospy_instance else rospy.logerr
    log_warn = rospy_instance.logwarn if rospy_instance else rospy.logwarn
    waypoints = [] # 存储生成的路径点列表
    # 输入参数验证
    if center_pose is None:
        log_err("center_pose cannot be None in generate_letter_waypoints_from_font.")
        return []
    # 提取中心位置的坐标分量
    base_x = center_pose.position.x # 机器人X轴位置（固定）
    offset_y_world = center_pose.position.y # Y轴偏移量
    offset_z_world = center_pose.position.z # Z轴偏移量
    fixed_orientation = center_pose.orientation # 固定的末端执行器姿态
    # 1: 加载字体文件
    try:
        face = freetype.Face(font_path)
    except freetype.FT_Exception as e:
        log_err(f"Failed to load font: {font_path}. Error: {e}")
        return []
    # 2: 设置字体像素大小（用于轮廓提取）
    face.set_pixel_sizes(0, 64)  # 64像素高度，宽度自动调整
    # 3: 加载指定字符的字形轮廓
    try:
        face.load_char(char_to_draw, freetype.FT_LOAD_NO_SCALE | freetype.FT_LOAD_NO_BITMAP)
    except freetype.FT_Exception as e:
        log_err(f"Failed to load glyph for char '{char_to_draw}'. Error: {e}")
        return []
    # 4: 获取字符轮廓并进行分解
    outline = face.glyph.outline
    decomposer = OutlineDecomposer(num_samples_per_curve_segment=num_samples_per_curve) 
    outline.decompose(decomposer, move_to=decomposer.move_to, line_to=decomposer.line_to, conic_to=decomposer.conic_to, cubic_to=decomposer.cubic_to)
    font_units_points = decomposer.points_2d_font_units
    # 检查是否成功提取到轮廓点
    if not font_units_points:
        log_warn(f"No points generated for char '{char_to_draw}'. It might be an empty glyph or decomposition issue.")
        return []
    # 5: 计算字符的边界框和缩放因子
    min_font_x = min(p[0] for p in font_units_points) # 字符左边界
    max_font_x = max(p[0] for p in font_units_points) # 字符右边界
    min_font_y = min(p[1] for p in font_units_points) # 字符下边界
    max_font_y = max(p[1] for p in font_units_points) # 字符上边界
    font_width_fu = max_font_x - min_font_x  # 字符宽度（字体单位）
    font_height_fu = max_font_y - min_font_y # 字符高度（字体单位）
    # 计算缩放因子
    if font_height_fu == 0:
        if font_width_fu == 0:
             log_warn(f"Character '{char_to_draw}' has zero width and height in font units.")
             scale_factor = 0.0
        else:
             scale_factor = desired_letter_height_m / font_width_fu if font_width_fu else 0.0
    else:
        scale_factor = desired_letter_height_m / font_height_fu
    # 计算中心点
    center_font_x = (min_font_x + max_font_x) / 2.0
    center_font_y = (min_font_y + max_font_y) / 2.0
    for fx, fy in font_units_points:
        local_y = (fx - center_font_x) * scale_factor
        local_z = (fy - center_font_y) * scale_factor
        pose = Pose()
        pose.position.x = base_x
        pose.position.y = offset_y_world + local_y
        pose.position.z = offset_z_world + local_z
        pose.orientation = copy.deepcopy(fixed_orientation)
        waypoints.append(pose)
    log_info(f"Generated {len(waypoints)} waypoints for char '{char_to_draw}' with height {desired_letter_height_m:.2f}m (from helper).")
    return waypoints