#!/usr/bin/env python3

import rospy
import socket
import actionlib
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from copy import deepcopy

class RobotiqActionServer(object):

    def __init__(self, namespace, action_name, robot_ip="10.0.0.1", port=63352):
        self._action_name = namespace + action_name
        self._action_server = actionlib.SimpleActionServer(self._action_name, 
            GripperCommandAction, 
            execute_cb=self.execute_cb, 
            auto_start = False)
        
        self._robot_ip = robot_ip
        if type(port) is int:
            self._port = port
        elif type(port) is str:
            self._port = int(port)
        else:
            raise Exception
        
        self.start_activation()
        rospy.loginfo("Robotiq activation started")
        rospy.sleep(5)
        
        self._action_server.start()
        rospy.loginfo("Robotiq server started")
    
    def send_command(self, cmd):
        if type(cmd) is str:
            cmd_str = deepcopy(cmd)
        elif type(cmd) is bytes:
            cmd_str = cmd.decode()
        else:
            raise Exception
        if not cmd_str.endswith("\n"):
            cmd_str += "\n"
        cmd_bytes = cmd_str.encode()

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self._robot_ip, self._port))
            s.sendall(cmd_bytes)
            data = s.recv(2**10)
        
        return data

    def start_activation(self):
        self.send_command("SET ACT 1\n")

    def set_pos(self, pos):
        self.send_command("SET POS {}\n".format(pos))
    
    def execute_cb(self, goal):
        
        self.set_pos(int(goal.command.position*255))
        
        result = GripperCommandResult()
  
        self._action_server.set_succeeded(result)  
      

if __name__ == "__main__":

    rospy.init_node('robotiq_action_server')

    # Get Node parameters
    robot_ip = rospy.get_param('~robot_ip','10.0.0.1')
    port = rospy.get_param('~port','63352')

    server = RobotiqActionServer(rospy.get_namespace(), 'gripper_controller/gripper_cmd', robot_ip=robot_ip, port=port)
    
    rospy.spin()
    
    
