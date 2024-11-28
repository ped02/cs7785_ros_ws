import math
import time
from scipy.spatial.transform import Rotation

import rclpy

import rclpy.time
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from wachakorn_chase_object.ros_utils import (
    rotation_to_quat_message,
)


def main():
    rclpy.init()
    nav = BasicNavigator()

    # ...

    # nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()  # if autostarted, else use lifecycleStartup()

    # ...

    # path = nav.getPath(init_pose, goal_pose)
    # smoothed_path = nav.smoothPath(path)

    # ...

    # goal_poses = [ (2.0, 0.0, 0.0), (1.5, 0.0, math.pi/2) ]

    # Real Map
    goal_poses = [
        (2.05, 0.05, math.pi / 2),
        (0.06, 1.0, 0.0),
        (1.75, 1.75, 0.0),
    ]

    # Virtual Map
    # goal_poses = [ (1.75, 0.0, 0.0), (4.4, 1.75, 0.0), (0.0, 1.75, -math.pi/2) ]

    for goal_pose in goal_poses:
        goal_pose_message = PoseStamped()
        goal_pose_message.header.frame_id = 'map'
        goal_pose_message.header.stamp = rclpy.time.Time().to_msg()

        goal_pose_message.pose.position.x = float(goal_pose[0])
        goal_pose_message.pose.position.y = float(goal_pose[1])

        goal_pose_message.pose.orientation = rotation_to_quat_message(
            Rotation.from_euler('xyz', [0.0, 0.0, goal_pose[2]])
        )

        nav.goToPose(goal_pose_message)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback.navigation_time.nanosec * 1e-9 > 10:
                nav.cancelTask()
            time.sleep(0.01)

        # ...

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

        time.sleep(2.0)
        print('next goal')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
