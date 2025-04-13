#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Int32
import json
from geometry_msgs.msg import PoseWithCovarianceStamped 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class SaveGoalNode(Node):
    def __init__(self):
    
        self.pre_save_goal = None
        self.save_goal = None

        self.save_goal_id = -1

        self.current_robot_pose_x = 0.0
        self.current_robot_pose_y = 0.0
        self.current_robot_pose_z = 0.0
        self.current_robot_pose_w = 0.0

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # This sets how many messages to keep
        )

        super().__init__("save_goal_node")
        self.save_goal_subscriber_ = self.create_subscription(
            Bool, "save_goal", self.saveGoalCallback, 10)
        self.save_goal_subscriber_ = self.create_subscription(
            Int32, "save_goal_id", self.saveGoalIdCallback, 10)
        self.robot_pose_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self.getRobotPoseCallback, 10)
        
        self.update_goal_list_pub_ = self.create_publisher(
            Bool, "update_goal_list", self.qos_profile
        )

        self.get_logger().info("SaveGoalNode has been started.")


    def saveGoalCallback(self, msg: Bool):
        self.save_goal = msg.data
    
    def saveGoalIdCallback(self, msg: Int32):
        if (self.save_goal_id != msg.data):
            self.save_goal_id = msg.data
            update_goal_list_pub_data = Bool()
            update_goal_list_pub_data.data = False
            self.update_goal_list_pub_.publish(update_goal_list_pub_data)

        if (self.pre_save_goal == None):
            self.pre_save_goal = self.save_goal
            update_goal_list_pub_data = Bool()
            update_goal_list_pub_data.data = False
            self.update_goal_list_pub_.publish(update_goal_list_pub_data)

        elif ((self.pre_save_goal != None) and (self.pre_save_goal != self.save_goal)):
            if (self.pre_save_goal == True) and (self.save_goal == False):
                self.updateGoalDatabase()
            self.pre_save_goal = self.save_goal
            
        else:
            update_goal_list_pub_data = Bool()
            update_goal_list_pub_data.data = False
            self.update_goal_list_pub_.publish(update_goal_list_pub_data)
    
    def updateGoalDatabase(self):
        json_file_path = "/home/tai/mybot_ws/src/MYBOT-ROS2/goal_database/goal_list.json"
        try:
            with open(json_file_path, 'r') as f:
                data = json.load(f)
            for goal in data['goals']:
                if goal['goal_id'] == self.save_goal_id:
                    goal['x'] = self.current_robot_pose_x
                    goal['y'] = self.current_robot_pose_y
                    goal['z'] = self.current_robot_pose_z
                    goal['w'] = self.current_robot_pose_w
                    break
                
            with open(json_file_path, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f"Goal ID {self.save_goal_id} updated with robot pose.")

            update_goal_list_pub_data = Bool()
            update_goal_list_pub_data.data = True
            self.update_goal_list_pub_.publish(update_goal_list_pub_data)

        except Exception as e:
            self.get_logger().error("Failed to update goal database")

    def getRobotPoseCallback(self, msg: PoseWithCovarianceStamped):
        self.current_robot_pose_x = msg.pose.pose.position.x
        self.current_robot_pose_y = msg.pose.pose.position.y
        self.current_robot_pose_z = msg.pose.pose.orientation.z
        self.current_robot_pose_w = msg.pose.pose.orientation.w

def main(args=None):
    rclpy.init(args=args)
    node = SaveGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()