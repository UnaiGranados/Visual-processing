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

def publish_transform(transf, base_link, child_link, time_stamp):
  br = tf2_ros.TransformBroadcaster()
  tf_msg = TransformStamped()
  tf_msg.header.stamp = time_stamp
  tf_msg.header.frame_id = base_link
  tf_msg.child_frame_id = child_link
  tf_msg.transform.translation.x = transf[0, 3]
  tf_msg.transform.translation.y = transf[1, 3]
  tf_msg.transform.translation.z = transf[2, 3]
  q = pyq.Quaternion(matrix=transf[0:3, 0:3])
  tf_msg.transform.rotation.w = q[0]
  tf_msg.transform.rotation.x = q[1]
  tf_msg.transform.rotation.y = q[2]
  tf_msg.transform.rotation.z = q[3]
  br.sendTransform(tf_msg)

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
    rospy.init_node('move_group_node', anonymous=True)

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
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = -0.5 * pi
    joint_goal[5] = -0.38 * pi

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    # current_pose = self.move_group.get_current_pose().pose
    # print ("Current pose:")
    # print ("- x : " + str(current_pose.position.x))
    # print ("- y : " + str(current_pose.position.y))
    # print ("- z : " + str(current_pose.position.z))
    # current_pose.position.x += 0.1
    # current_pose.position.z -= 0.1
    # move_group.set_pose_target(current_pose)
    # plan = move_group.go(wait=True)
    # move_group.stop()
    # current_pose = self.move_group.get_current_pose().pose
    # print ("Current pose:")
    # print ("- x : " + str(current_pose.position.x))
    # print ("- y : " + str(current_pose.position.y))
    # print ("- z : " + str(current_pose.position.z))

    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, transform):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    current_pose = self.move_group.get_current_pose().pose
    print ("Current pose:")
    print ("- x : " + str(current_pose.position.x))
    print ("- y : " + str(current_pose.position.y))
    print ("- z : " + str(current_pose.position.z))
    print ("- qw : " + str(current_pose.orientation.w))
    print ("- qx : " + str(current_pose.orientation.x))
    print ("- qy : " + str(current_pose.orientation.y))
    print ("- qz : " + str(current_pose.orientation.z))

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

    print ("Goal pose:")
    print ("- x : " + str(pose_goal.position.x))
    print ("- y : " + str(pose_goal.position.y))
    print ("- z : " + str(pose_goal.position.z))
    print ("- qw : " + str(pose_goal.orientation.w))
    print ("- qx : " + str(pose_goal.orientation.x))
    print ("- qy : " + str(pose_goal.orientation.y))
    print ("- qz : " + str(pose_goal.orientation.z))

    move_group.set_pose_target(pose_goal)
    print(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    
    if current_pose.position.x == pose_goal.position.x and current_pose.position.y == pose_goal.position.y and current_pose.position.z == pose_goal.position.z :

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

    tutorial.go_to_joint_state()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # rospy.wait_for_service("tag_results")
    # number_tags_sp=rospy.ServiceProxy("tag_results", ReturnNumberTags)

    callback_lambda = lambda x: timer_callback(x, tutorial, tfBuffer)
    
    rospy.Timer(rospy.Duration(1), callback_lambda)  
    rospy.spin()
   
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


def timer_callback(event, move_group_obj, tfBuffer):
  print ('Timer called at ' + str(event.current_real))
  
  # Number_tags=number_tags_sp()

  # print("Tag results:" + str(Number_tags.n_tags))

  # if Number_tags.n_tags >=1:
  #   i = 1
  # for i in range(1, Number_tags.n_tags + 1):
    
  try:   
    
    #Read tf transformations
    base_T_tag = tfBuffer.lookup_transform("base_link", "tag_frame1", rospy.Time.now(), rospy.Duration(2))   
    base_T_tool0 = tfBuffer.lookup_transform("base_link", "tool0", rospy.Time.now(), rospy.Duration(2)) 
    world_T_base = tfBuffer.lookup_transform("world", "base_link", rospy.Time.now(), rospy.Duration(2))   
    T_distance = tfBuffer.lookup_transform("tool0", "tag_frame1", rospy.Time.now(), rospy.Duration(2))
    world_T_opt = tfBuffer.lookup_transform("world","camera_color_optical_frame",  rospy.Time.now(), rospy.Duration(2))
    tool0_T_opt = tfBuffer.lookup_transform("tool0","camera_color_optical_frame",  rospy.Time.now(), rospy.Duration(2))
    
    print(str(base_T_tool0)) 
    
    #Compute translations
    base_P_tag = np.array([[base_T_tag.transform.translation.x],[base_T_tag.transform.translation.y],[base_T_tag.transform.translation.z],[1]])
    base_P_tool0 =  np.array([[base_T_tool0.transform.translation.x],[base_T_tool0.transform.translation.y],[base_T_tool0.transform.translation.z],[1]])
    world_P_base = np.array([[world_T_base.transform.translation.x],[world_T_base.transform.translation.y],[world_T_base.transform.translation.z],[1]])
    P_distance = np.array([[T_distance.transform.translation.x],[T_distance.transform.translation.y],[T_distance.transform.translation.z],[1]])  
    world_P_opt = np.array([[world_T_opt.transform.translation.x],[world_T_opt.transform.translation.y],[world_T_opt.transform.translation.z],[1]])
    tool0_P_opt = np.array([[tool0_T_opt.transform.translation.x],[tool0_T_opt.transform.translation.y],[tool0_T_opt.transform.translation.z],[1]])

    #Compute rotations
    tag_R_base = pyq.Quaternion(np.array([base_T_tag.transform.rotation.w,base_T_tag.transform.rotation.x,base_T_tag.transform.rotation.y,base_T_tag.transform.rotation.z]))
    tool0_R_base = pyq.Quaternion(np.array([base_T_tool0.transform.rotation.w,base_T_tool0.transform.rotation.x,base_T_tool0.transform.rotation.y,base_T_tool0.transform.rotation.z]))  
    world_R_base = pyq.Quaternion(np.array([world_T_base.transform.rotation.w,world_T_base.transform.rotation.x,world_T_base.transform.rotation.y,world_T_base.transform.rotation.z]))
    R_distance = pyq.Quaternion(np.array([T_distance.transform.rotation.w,T_distance.transform.rotation.x,T_distance.transform.rotation.y,T_distance.transform.rotation.z]))
    world_R_opt = pyq.Quaternion(np.array([world_T_opt.transform.rotation.w,world_T_opt.transform.rotation.x,world_T_opt.transform.rotation.y,world_T_opt.transform.rotation.z]))
    tool0_R_opt = pyq.Quaternion(np.array([tool0_T_opt.transform.rotation.w,tool0_T_opt.transform.rotation.x,tool0_T_opt.transform.rotation.y,tool0_T_opt.transform.rotation.z]))
    
    #Transformation matrix
    base_T_tag = tag_R_base.transformation_matrix
    base_T_tool0 = tool0_R_base.transformation_matrix
    world_T_base = world_R_base.transformation_matrix
    T_distance =  R_distance.transformation_matrix
    world_T_opt = world_R_opt.transformation_matrix
    tool0_T_opt = tool0_R_opt.transformation_matrix
    base_T_tag[:,3]=base_P_tag.T
    base_T_tool0[:,3]=base_P_tool0.T
    world_T_base[:,3]=world_P_base.T
    T_distance[:,3]=P_distance.T
    world_T_opt[:,3]=world_P_opt.T
    tool0_T_opt[:,3]=tool0_P_opt.T
    
    #Calculate distance between tag and tool
    # distance_to_tag= np.sqrt(mat_distance[0,3] + mat_distance[1,3] + mat_distance[2,3])
    # pub=rospy.Publisher("/distance_to_tag", Double, queue_size=10)
    # pub.publish(distance_to_tag)

    print(Fore.YELLOW + "Transformation between tag and base_link is:" + str(base_T_tag))
    print("Transformation between tool0 and base_link is:" + str(base_T_tool0))
    print("Transformation between world and base_link is:" + str(world_T_base) + Style.RESET_ALL)

    #Create an offset matrix
    tag_T_goal= np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.5], [0, 0, 0, 1]])
    print("Matriz de offset:" + str(tag_T_goal))

    #Calculate transformation matrix between world and goal and move robot to goal
    base_T_goal=np.dot(base_T_tag, tag_T_goal)
    base_T_goal_no_rotation = np.eye(4)
    base_T_goal_no_rotation[:3, :3] = base_T_tool0[:3, :3]
    base_T_goal_no_rotation[:3, 3] = base_T_goal[:3, 3]
    world_T_goal = np.dot(world_T_base,base_T_goal_no_rotation)
    print(Fore.GREEN + "Transformation between goal and world is:" + str(world_T_goal) + Style.RESET_ALL)
    t_goal_tag =  publish_transform(tag_T_goal,"tag_frame1", "goal", rospy.Time.now())
    
    world_T_tool0=np.dot(world_T_opt,tool0_T_opt)
    print(Fore.GREEN + "Transformation between opt and world is:" + str(world_T_opt) + Style.RESET_ALL)
    print(Fore.GREEN + "Transformation between opt and tool is:" + str(tool0_T_opt) + Style.RESET_ALL)
    print(Fore.GREEN + "Transformation between tool and world is:" + str(world_T_tool0) + Style.RESET_ALL)

    opt_T_goal=np.dot(world_T_opt, np.linalg.inv(world_T_goal))
    opt_T_goal[:3, :3] = world_T_opt[:3, :3]
    opt_T_goal[:3, 3] = world_T_goal[:3, 3]
    
    world_T_tool = np.dot(opt_T_goal, np.linalg.inv(tool0_T_opt))
    move_group_obj.go_to_pose_goal(world_T_tool)
  
    print(Fore.RED + "Transformation between goal and base is:" + str(base_T_goal) + Style.RESET_ALL)
    print(Fore.YELLOW + "Transformation between goal and world is:" + str(world_T_goal) + Style.RESET_ALL)
    print ("============ Move group completed!=============")

  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print("Transform not available")
  
  # i=i+1
    
if __name__ == '__main__':
  main()
 
  
 


