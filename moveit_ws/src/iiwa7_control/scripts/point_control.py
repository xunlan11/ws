#!/usr/bin/env python3
# coding: utf-8

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import tau
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker

class PointControl:
    def __init__(self):
        # 初始化moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化rospy节点
        rospy.init_node('iiwa7_point_to_point_control', anonymous=True)
        # 实例化RobotCommander对象，提供机器人运动学模型和关节状态信息
        self.robot = moveit_commander.RobotCommander()
        # 实例化PlanningSceneInterface对象，提供获取、设置和更新机器人对周围世界内部理解的远程接口
        self.scene = moveit_commander.PlanningSceneInterface()
        # 实例化MoveGroupCommander对象，接入运动规划组manipulator
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        # Rviz轨迹发布者
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', # 话题名称
            moveit_msgs.msg.DisplayTrajectory,  # 消息类型
            queue_size=20                       # 队列大小
            )                          
        # Rviz轨迹可视化
        self.marker_publisher = rospy.Publisher(
            '/visualization_marker',
            Marker,
            queue_size=10
            )
        # 规划参考坐标系
        self.planning_frame = self.move_group.get_planning_frame()
        rospy.loginfo("机器人参考坐标系: %s" % self.planning_frame)
        # 末端执行器链接
        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("末端执行器: %s" % eef_link)
        # 机器人组
        group_names = self.robot.get_group_names()
        rospy.loginfo("机器人组: %s" % group_names)
        # 当前状态
        initial_state = self.move_group.get_current_state()
        rospy.loginfo("初始当前状态: %s" % initial_state)
        # 等待连接
        rospy.loginfo("等待连接到PlanningSceneInterface。")
        rospy.wait_for_service('/get_planning_scene', timeout=10) 
        rospy.loginfo("成功连接到PlanningSceneInterface。")
    # 添加盒子障碍物
    def add_obstacle_box(self, name, pose_stamped, size=(0.1, 0.1, 0.1)):
        self.scene.add_box(name, pose_stamped, size)
        rospy.loginfo(f"添加障碍物 '{name}' ，位置: {pose_stamped.pose.position}, 大小: {size}。")
        start_time = rospy.Time.now()
        seconds_to_wait = 5.0
        while (rospy.Time.now() - start_time).to_sec() < seconds_to_wait:
            is_known = name in self.scene.get_known_object_names()
            if is_known:
                rospy.loginfo(f"障碍物 '{name}' 已成功添加。")
                return True
            rospy.sleep(0.1)
        rospy.logwarn(f"障碍物 '{name}' 添加超时，可能未成功。")
        return False
    # 移除障碍物
    def remove_obstacle(self, name):
        self.scene.remove_world_object(name)
        rospy.loginfo(f"请求移除障碍物 '{name}' 中。")
        start_time = rospy.Time.now()
        seconds_to_wait = 5.0 
        while (rospy.Time.now() - start_time).to_sec() < seconds_to_wait:
            is_known = name in self.scene.get_known_object_names()
            if not is_known:
                rospy.loginfo(f"障碍物 '{name}' 已成功移除。")
                return True
            rospy.sleep(0.1)
        rospy.logwarn(f"障碍物 '{name}' 移除超时，可能未成功。")
        return False
    # 移动到指定关节角度
    def move_J(self, joint_goal_array):
        # 判断给定关节角度的数量是否合理
        current_joints = self.move_group.get_current_joint_values()
        if len(joint_goal_array) != len(current_joints):    
            rospy.logerr(f"目标关节数 {len(joint_goal_array)} 与实际关节数 {len(current_joints)} 不符。")
            return False
        self.move_group.set_joint_value_target(joint_goal_array)
        # 规划
        plan_success, plan, _, _ = self.move_group.plan()
        if not plan_success:
            rospy.logerr(f"规划目标角度失败。")
            return False
        rospy.loginfo(f"规划目标角度成功，开始执行。")
        # 执行
        execute_success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        if not execute_success:
            rospy.logerr("执行目标角度失败。")
            return False
        rospy.loginfo("执行目标角度成功。")
        return True
    # 规划笛卡尔路径
    def plan_cartesian_path(self, waypoints):
        # 较低的最大速度和加速度缩放因子，确保成功且平滑
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        waypoints_list = copy.deepcopy(waypoints)
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints_list,  # 路径点列表
            0.01,            # 末端执行器步长
        )
        rospy.loginfo(f"笛卡尔路径规划完成，成功比例: {fraction:.2f}。")
        return plan, fraction
    # 执行规划轨迹
    def execute_plan(self, plan):
        rospy.loginfo("开始执行规划的轨迹。")
        success = self.move_group.execute(plan, wait=True)
        if success:
            rospy.loginfo("轨迹执行成功。")
        else:
            rospy.logerr("轨迹执行失败。")
        return success
    # 清除指定空间下的所有RViz Markers
    def clear_markers(self, ns="trajectory"):
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0 
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(marker)
        rospy.loginfo(f"已发送清除Marker命令到空间{ns}")
    # 显示路径
    def show_path(self, waypoints_pose_list, ns="trajectory", r=1.0, g=0.0, b=0.0):
        # 清除空间中的旧Markers
        self.clear_markers(ns)
        rospy.sleep(0.05)
        # 创建并发布新Marker
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015 # 线条宽度
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.8 # 不透明度
        marker.lifetime = rospy.Duration()
        for pose in waypoints_pose_list:
            marker.points.append(pose.position)
        rospy.loginfo(f"发布包含 {len(marker.points)} 个点的轨迹 Marker 到 /eef_trajectory_marker (ns: {ns})。")
        self.marker_publisher.publish(marker)
    # 五角星(XY平面)
    def run(self):
        # 清除障碍物
        rospy.loginfo("============ 清除所有已存在的障碍物 ============")
        known_object_names = self.scene.get_known_object_names()
        if known_object_names:
            rospy.loginfo(f"场景中发现已存在的障碍物: {known_object_names}，正在清除。")
            for obj_name in known_object_names:
                self.remove_obstacle(obj_name)
            rospy.loginfo("所有障碍物清除完毕。")
            rospy.sleep(1.0)
        else:
            rospy.loginfo("场景中没有障碍物。")
        self.clear_markers()
        # 初始姿态：非奇异
        rospy.loginfo("============ 移动到初始姿态 ============")
        ready_joint_angles = [0.0, np.deg2rad(-15), 0.0, np.deg2rad(-90), 0.0, np.deg2rad(90), 0.0]
        if not self.move_J(ready_joint_angles):
            rospy.logerr("移动到初始姿态失败，中止。")
            return
        rospy.loginfo("成功移动到初始姿态，请按回车键继续。")
        input()
        # 获取当前姿态
        current_pose_msg = self.move_group.get_current_pose()
        center = current_pose_msg.pose
        # 定义五角星轨迹
        radius = 0.18  # 外接圆半径 (米)
        # 末端执行器在各顶点的姿态与初始姿态一致
        eef_msg = center.orientation
        eef_orientation = [eef_msg.x, eef_msg.y, eef_msg.z, eef_msg.w]
        # 局部坐标系（五角星中心为原点）到世界坐标系的变换矩阵
        # 旋转
        local2world = quaternion_matrix(eef_orientation)
        # 平移
        local2world[0,3] = center.position.x
        local2world[1,3] = center.position.y
        local2world[2,3] = center.position.z
        points_data = []
        # 五角星顶点索引顺序 (轨迹是按正常画五角星的顺序)
        star_point_indices = [0, 2, 4, 1, 3]
        # 计算各顶点位姿
        for i in range(5):
            # 角度
            angle_factor = star_point_indices[i]
            angle = (tau / 5) * angle_factor # tau是2*pi
            # 坐标（局部坐标系XY平面）
            x_local = radius * np.cos(angle)
            y_local = radius * np.sin(angle)
            z_local = 0.0
            p_local = np.array([x_local, y_local, z_local, 1.0])
            # 转换到世界坐标系
            p_world = local2world @ p_local
            vertex_world_x = p_world[0]
            vertex_world_y = p_world[1]
            vertex_world_z = p_world[2]
            points_data.append([vertex_world_x, vertex_world_y, vertex_world_z] + eef_orientation)
        # 转换位姿对象，生成路径点列表（回到起点）
        waypoints = []
        for point_data in points_data:
            pose = Pose()
            pose.position.x = point_data[0]
            pose.position.y = point_data[1]
            pose.position.z = point_data[2]
            pose.orientation.x = point_data[3]
            pose.orientation.y = point_data[4]
            pose.orientation.z = point_data[5]
            pose.orientation.w = point_data[6]
            waypoints.append(copy.deepcopy(pose))
        pose = Pose()
        point_data = points_data[0]
        pose.position.x = point_data[0]
        pose.position.y = point_data[1]
        pose.position.z = point_data[2]
        pose.orientation.x = point_data[3]
        pose.orientation.y = point_data[4]
        pose.orientation.z = point_data[5]
        pose.orientation.w = point_data[6]
        waypoints.append(copy.deepcopy(pose))
        self.show_path(waypoints)
        # 规划
        rospy.loginfo("============ 规划笛卡尔路径 (无障碍物) ============")
        plan_no_obs, fraction_no_obs = self.plan_cartesian_path(copy.deepcopy(waypoints))
        rospy.loginfo("规划成功比例: %.2f。" % fraction_no_obs)
        if fraction_no_obs < 0.9:
            rospy.logwarn("路径规划不完整。")
        display_trajectory_no_obs = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory_no_obs.trajectory_start = self.robot.get_current_state()
        display_trajectory_no_obs.trajectory.append(plan_no_obs)
        self.display_trajectory_publisher.publish(display_trajectory_no_obs)
        rospy.loginfo("完成轨迹规划，按回车键执行。")
        input()
        # 执行
        rospy.loginfo("============ 执行轨迹 (无障碍物) ============")
        if fraction_no_obs > 0.01:
            success_no_obs = self.execute_plan(plan_no_obs)
            rospy.loginfo("轨迹执行 " + ("成功。" if success_no_obs else "失败。"))
        else:
            rospy.loginfo("轨迹规划成功比例过低，不执行。")
        rospy.loginfo("演示完成。按回车键回到初始位置并继续。")
        input()
        
        # 有障碍物
        # 回到初始姿态
        rospy.loginfo("============ 移动到初始姿态 ============")
        ready_joint_angles = [0.0, np.deg2rad(-15), 0.0, np.deg2rad(-90), 0.0, np.deg2rad(90), 0.0]
        if not self.move_J(ready_joint_angles):
            rospy.logerr("移动到初始姿态失败，中止。")
            return
        rospy.loginfo("成功移动到初始姿态，请按回车键继续。")
        input()
        # 添加障碍物
        obstacle_name = "obstacle_box_star"
        obstacle_pose = geometry_msgs.msg.PoseStamped()
        obstacle_pose.header.frame_id = self.planning_frame
        # 计算障碍物位置（前两个绘制点中点）
        if len(waypoints) >= 2: # 保证轨迹至少有两个点
            p0 = waypoints[0].position
            p1 = waypoints[1].position
            # 无法躲避障碍物
            #obstacle_pose.pose.position.x = (p0.x + p1.x) / 2 + 0.1
            #obstacle_pose.pose.position.y = (p0.y + p1.y) / 2 + 0.1
            #obstacle_pose.pose.position.z = (p0.z + p1.z) / 2
            # 可躲避障碍物
            obstacle_pose.pose.position.x = (p0.x + p1.x) / 2 - 0.4
            obstacle_pose.pose.position.y = (p0.y + p1.y) / 2 + 0.18
            obstacle_pose.pose.position.z = (p0.z + p1.z) / 2
            obstacle_pose.pose.orientation.w = 1.0
            obstacle_size = (0.1, 0.01, 0.1) # 障碍物尺寸
            # 避免重复添加（删除已有的）
            if obstacle_name in self.scene.get_known_object_names():
                self.remove_obstacle(obstacle_name)
                rospy.sleep(0.5)
            self.add_obstacle_box(obstacle_name, obstacle_pose, obstacle_size)
            rospy.sleep(0.5)
        else:
            rospy.logwarn("路径点不足2个，无法计算障碍物位置。")
        # 规划
        rospy.loginfo("============ 规划笛卡尔路径 (有障碍物) ============")
        plan_obs, fraction_obs = self.plan_cartesian_path(copy.deepcopy(waypoints))
        rospy.loginfo("规划成功比例: %.2f。" % fraction_obs)
        # 是否能部分规划
        if fraction_obs < 0.9:
            rospy.logwarn("路径规划不完整。")
            if fraction_obs < 0.01:
                 rospy.logerr("无法规划任何有效路径。")
        display_trajectory_obs = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory_obs.trajectory_start = self.robot.get_current_state()
        display_trajectory_obs.trajectory.append(plan_obs)
        self.display_trajectory_publisher.publish(display_trajectory_obs)
        rospy.loginfo("完成轨迹规划，按回车键执行。")
        input()
        # 执行
        rospy.loginfo("============ 执行轨迹 (有障碍物) ============")
        if fraction_obs > 0.01:
            success_obs = self.execute_plan(plan_obs)
            if success_obs:
                if fraction_obs < 0.9:
                    rospy.loginfo(f"轨迹部分执行成功 (规划比例: {fraction_obs:.2f})。")
                else:
                    rospy.loginfo("轨迹执行成功。")
            else:
                rospy.logerr("轨迹执行失败。")
        else:
            rospy.loginfo("轨迹规划成功比例过低或无法规划，不执行。")
        # 清理障碍物
        if obstacle_name in self.scene.get_known_object_names():
            self.remove_obstacle(obstacle_name)
            rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        controller = PointControl()
        rospy.loginfo("初始化完成，控制器已就绪，按回车键继续。")
        input()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass