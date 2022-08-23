#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from multiprocessing.connection import wait
from pickle import FALSE
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# from pymoveit2.pymoveit2.moveit2_ur10e_1 import MoveIt2_ur10e_1
# from pymoveit2 import MoveIt2
#from pymoveit2 import MoveIt2Servo

from pymoveit2 import MoveIt2_ur10e_1

from pymoveit2.robots import ur10e_1

# from pymoveit2.pymoveit2.robots import ur10e_1

from math import degrees, radians, cos, sin

import time

def main(args=None):

    rclpy.init(args=args)

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.2, 0.2, 1.0])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2_ur10e_1(
        node=node,
        joint_names=ur10e_1.joint_names(),
        base_link_name=ur10e_1.base_link_name(),
        end_effector_name=ur10e_1.end_effector_name(),
        group_name=ur10e_1.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"BEVEGELSE!!!!!!"
    )
    
    moveit2.max_velocity = 0.2
    moveit2.max_acceleration = 0.1
    
    conf_test = [0.0, -0.6, -2.1, -1.3, -4.7, 0.0]
    
    point_1 = [-0.5, -0.6, 0.6]
    
    point_2 = [-0.5, -0.6, 0.8]
    conf_2 = [radians(-117), radians(-98), radians(-69), radians(-103), radians(90), radians(-27)]
    
    point_3 = [+0.5, 0.7, 0.8]
    conf_3 = [radians(66), radians(-106), radians(-59), radians(-105), radians(90), radians(-24)]
    
    point_4 = [+0.5, 0.7, 0.6]
    
    # point_1 = [1.2, 0.0, 0.2]
    # point_2 = [1.0, 0.0, 0.4]
    # point_3 = [0.8, 0.0, 0.6]
    # point_4 = [0.6, 0.0, 0.8]
    
    Quat_1 = [0.0, 1.0, 0.0, 0.0]
    Quat_2 = [1.0, 0.0, 0.0, 0.0]
    Quat_3 = [1.0, 0.0, 0.0, 0.0]
    Quat_4 = [1.0, 0.0, 0.0, 0.0]

    
    # path_joint_constraints_virker = [
    #         (ur10e_1.joint_names()[0], radians(-190), radians(-180), radians(180)),
    #         (ur10e_1.joint_names()[1], radians(-90), radians(-45), radians(45)),
    #         (ur10e_1.joint_names()[2], radians(0), radians(-1), radians(170)),
    #         (ur10e_1.joint_names()[3], radians(-90), radians(-90), radians(90)),
    #         (ur10e_1.joint_names()[4], radians(-90), radians(-90), radians(90)),
    #     ]
    
    # path_joint_constraints_1 = [
    #         (ur10e_1.joint_names()[0], radians(-190), radians(-180), radians(180)),
    #         (ur10e_1.joint_names()[1], radians(-90), radians(-45), radians(45)),
    #         (ur10e_1.joint_names()[2], radians(0), radians(-1), radians(170)),
    #         (ur10e_1.joint_names()[3], radians(-90), radians(-90), radians(90)),
    #         (ur10e_1.joint_names()[4], radians(-90), radians(-90), radians(90)),
    #         (ur10e_1.joint_names()[5], radians(-190), radians(-180), radians(180)),
    #     ]
    
    # funker men bare hvis jeg fjerner constraints for en jojnt
    path_joint_constraints_1 = [
            (ur10e_1.joint_names()[0], radians(0), radians(-180), radians(180)), #shoulder_pan_joint
            (ur10e_1.joint_names()[1], radians(0), radians(-180), radians(0)), #shoulder_lift_joint
            #(ur10e_1.joint_names()[2], radians(0), radians(-150), radians(0)), #elbow_joint
            (ur10e_1.joint_names()[3], radians(0), radians(-140), radians(50)), #wrist_1_joint
            (ur10e_1.joint_names()[4], radians(0), radians(0), radians(180)), #wrist_2_joint
            (ur10e_1.joint_names()[5], radians(0), radians(-180), radians(180)), #wrist_3_joint
        ]

        
    # # alt
    # path_joint_constraints_1 = [
    #         (ur10e_1.joint_names()[0], radians(0), radians(-3600), radians(3600)),
    #         (ur10e_1.joint_names()[1], radians(0), radians(-3600), radians(3600)),
    #         (ur10e_1.joint_names()[2], radians(0), radians(-3600), radians(3600)),
    #         (ur10e_1.joint_names()[3], radians(0), radians(-3600), radians(3600)),
    #         (ur10e_1.joint_names()[4], radians(0), radians(-3600), radians(3600)),
    #         (ur10e_1.joint_names()[5], radians(0), radians(-3600), radians(3600)),
    #     ]
    
    # andrija
    # path_joint_constraints_1 = [
    #         (ur10e_1.joint_names()[0], radians(0), radians(-90), radians(90)),
    #         (ur10e_1.joint_names()[1], radians(-90), radians(-130), radians(-40)),
    #         (ur10e_1.joint_names()[2], radians(0), radians(-50), radians(50)),
    #         (ur10e_1.joint_names()[3], radians(0), radians(-100), radians(100)),
    #         (ur10e_1.joint_names()[4], radians(0), radians(-180), radians(180)),
    #     ]
    
    
    # # test
    # path_joint_constraints_1 = [
    #         (ur10e_1.joint_names()[0], radians(11), radians(-4), radians(4)),
    #     ]

    
    for i in range(10):
        print(i, ". runde, går til point1")
        moveit2.move_to_pose(position = point_1, quat_xyzw = Quat_1, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
        #time.sleep(2)
        moveit2.wait_until_executed()
    
        print( i, ". runde, går til point2")
        moveit2.move_to_configuration(conf_2)
        #moveit2.move_to_pose(position = point_2, quat_xyzw = Quat_1, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
        # #time.sleep(2)
        moveit2.wait_until_executed()

        print(i, ". runde, går til point3")
        moveit2.move_to_configuration(conf_3)
        #moveit2.move_to_pose(position = point_3, quat_xyzw = Quat_2, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
        # #time.sleep(2)
        moveit2.wait_until_executed()

        print(i, ". runde, går til point4")
        moveit2.move_to_pose(position = point_4, quat_xyzw = Quat_2, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
        # #time.sleep(2)
        moveit2.wait_until_executed()

        print(i, ". runde, går til point3")
        moveit2.move_to_configuration(conf_3)
        #moveit2.move_to_pose(position = point_3, quat_xyzw = Quat_2, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
        # #time.sleep(2)
        moveit2.wait_until_executed()

        print(i, ". runde, går til point2")
        moveit2.move_to_configuration(conf_2)
        #moveit2.move_to_pose(position = point_2, quat_xyzw = Quat_1, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
    # #time.sleep(2)
        moveit2.wait_until_executed()
    
    
    print("går til point1")
    moveit2.move_to_pose(position = point_1, quat_xyzw = Quat_1, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
    # #time.sleep(2)
    moveit2.wait_until_executed()
    

    
    
    #moveit2.move_to_pose(position = point_1, quat_xyzw = Quat_1, path_joint_constraints= path_joint_constraints_1, frame_id ="base")
    moveit2.move_to_configuration(conf_2)
    
    # moveit2.max_velocity = 1.0
    # moveit2.max_acceleration = 1.0
    
    #plan_1 = []
    #plan_1.append(moveit2.plan([0.5, 0.0, 0.25], [1.0, 0.0, 0.0, 0.0]))
    #plan_1.append(moveit2.plan([0.9, 0.0, 0.25], [1.0, 0.0, 0.0, 0.0]))
    #moveit2.execute(plan_1[0])
    
    
    
    
    #moveit2.move_to_pose([0.5, 0.0, 0.25], [1.0, 0.0, 0.0, 0.0])
    #MoveIt2_ur10e_1.move_to_pose([0.5, 0.0, 0.25], [1.0, 0.0, 0.0, 0.0])
    
    #moveit2.move_to_configuration(conf_1)
    
    
    # moveit2.move_to_pose(point_4, Quat_1,path_joint_constraints_1,frame_id="base")
    
    
    #moveit2._plan_cartesian_path([0.5, 0.0, 0.25], [1.0, 0.0, 0.0, 0.0])  #virket ikke

    #MoveIt2.set_joint_goal([0.0, -0.6, -2.1, -1.3, -4.7, 0.0],[
    #     'shoulder_pan_joint',
    #     'shoulder_lift_joint',
    #     'elbow_joint',
    #     'wrist_1_joint',
    #     'wrist_2_joint',
    #     'wrist_3_joint' ]) # virket ikke
    
    

    #moveit2.move_to_configuration(conf_1)
    # time.sleep(5)
    # #moveit2.wait_until_executed()
    # moveit2.move_to_configuration(conf_2)
    # time.sleep(5)
    # #moveit2.wait_until_executed()
    #moveit2.move_to_configuration(conf_3)
    
    #MoveIt2Servo.servo([0.5 ,0.5, 1.5],[0, 0, 0]) # virker ikke
    
    
    # # Create MoveIt 2 Servo interface
    # moveit2_servo = MoveIt2Servo(
    #     node=node,
    #     frame_id=ur10e_1.base_link_name(),
    #     callback_group=callback_group,
    # )
    # now_sec = node.get_clock().now().nanoseconds * 1e-9
    # moveit2_servo(linear=(sin(now_sec), cos(now_sec), 0.0), angular=(0.0, 0.0, 0.0))
    
    
    # plan_1 = moveit2.plan(joint_positions = conf_1, cartesian = True)
    # moveit2.execute(plan_1)
    
    
    
    
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
