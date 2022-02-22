#!/usr/bin/env python

from ast import Return
import sys
import copy
from tokenize import Double
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from  geometry_msgs.msg import Pose
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_ros
from tf2_ros import TransformListener
from tf2_ros import Buffer
import tf
from rospy import Time
from colorama import Fore, Back, Style
import pyquaternion as pyq
from geometry_msgs.msg import TransformStamped
from visual_processing.srv import  ReturnNumberTags, ReturnNumberTagsResponse

def all_close(goal, actual, tolerance):
  
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group iresultsme of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print ("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.1*pi
    joint_goal[1] = 0.1*pi
    joint_goal[2] = 0.02*pi
    joint_goal[3] = 0.1*pi
    joint_goal[4] = 0.1*pi
    joint_goal[5] = 0.75*pi

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, transform):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    quat = pyq.Quaternion(matrix=transform[0:4, 0:4])

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = quat[0]
    pose_goal.orientation.x = quat[1]
    pose_goal.orientation.y = quat[2]
    pose_goal.orientation.z = quat[3]
    pose_goal.position.x = transform[0, 3]
    pose_goal.position.y = transform[1, 3]
    pose_goal.position.z = transform[2, 3]

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


def main():

  try:
    tutorial = MoveGroupPythonIntefaceTutorial()

    # print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    # raw_input()
    tutorial.go_to_joint_state()

    # print ("============ Press `Enter` to execute a movement using a pose goal ...")
    # raw_input()
    # tutorial.go_to_pose_goal()

    print ("============ Move group completed!=============")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    callback_lambda = lambda x: timer_callback(x, tutorial, tfBuffer)
    rospy.Timer(rospy.Duration(3), callback_lambda)  
    rospy.spin()
  
   
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

def timer_callback(event, move_group_obj, tfBuffer):
  print ('Timer called at ' + str(event.current_real))
  # rospy.init_node('tf_listener')
  
  i=1

  rospy.wait_for_service("tag_results")
  Number_tags=rospy.ServiceProxy("Tag_results", ReturnNumberTags)
  print("Tag results:" + str(Number_tags))

  # for r in Number_tags:
    
  try:   
    
    trans = tfBuffer.lookup_transform("base_link", "tag_frame" + str(i), rospy.Time.now(), rospy.Duration(2))   
    trans_current_pose = tfBuffer.lookup_transform("base_link", "tool0", rospy.Time.now(), rospy.Duration(2)) 

    print(str(trans)) 
    
    x_t=(trans.transform.translation.x)
    y_t=(trans.transform.translation.y)
    z_t=(trans.transform.translation.z)
    w_q=(trans.transform.rotation.w)
    x_q=(trans.transform.rotation.x)
    y_q=(trans.transform.rotation.y)
    z_q=(trans.transform.rotation.z)

    #rotation_quaternion=np.array([w_q,x_q,y_q,z_q])
    translation = np.array([[x_t],[y_t],[z_t],[1]])
    translation_current_pose =  np.array([[trans_current_pose.transform.translation.x],[trans_current_pose.transform.translation.y],[trans_current_pose.transform.translation.z],[1]])
    #mat=pyq.Quaternion.transformation_matrix(rotation_quaternion)     
    rotation_quaternion = pyq.Quaternion(np.array([trans_current_pose.transform.rotation.w,trans_current_pose.transform.rotation.x,trans_current_pose.transform.rotation.y,trans_current_pose.transform.rotation.z]))  
    rotation_quaternion_current_pose = pyq.Quaternion(np.array([w_q,x_q,y_q,z_q]))
    mat = rotation_quaternion.transformation_matrix
    mat_current_pose = rotation_quaternion_current_pose.transformation_matrix
    mat[:,3]=translation.T
    mat_current_pose[:,3]=translation_current_pose.T
    distance_to_tag= np.sqrt(mat_current_pose[0,3] + mat_current_pose[1,3] + mat_current_pose[2,3])
    # pub=rospy.Publisher("/distance_to_tag", Double, queue_size=10)
    # pub.publish(distance_to_tag)
    print("Transformation between tag and base_link is:" + str(mat))
    Trans_goal_tag= np.matrix("1 0 0 0; 0 1 0 0; 0 0 1 0.5; 0 0 0 1")
    print("Matriz de offset:" + str(Trans_goal_tag))
    
    Trans_goal_base=mat * Trans_goal_tag
    print(Fore.GREEN + "Transformation between goal and base is:" + str(Trans_goal_base) + Style.RESET_ALL)

    
    move_group_obj.go_to_pose_goal(Trans_goal_base)
    print ("============ Move group completed!=============")

  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print("Transform not available")

  # i=i+1
  
  
    
if __name__ == '__main__':
  main()
 
  
 


