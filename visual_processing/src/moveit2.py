 try:   
    
    #Read tf transformations
    trans = tfBuffer.lookup_transform("base_link", "tag_frame1", rospy.Time.now(), rospy.Duration(2))   
    trans_current_pose = tfBuffer.lookup_transform("base_link", "tool0", rospy.Time.now(), rospy.Duration(2)) 
    trans_base_link = tfBuffer.lookup_transform("world", "base_link", rospy.Time.now(), rospy.Duration(2))   
    trans_distance = tfBuffer.lookup_transform("tool0", "tag_frame1", rospy.Time.now(), rospy.Duration(2))
    print(str(trans_current_pose)) 
    
    #Compute translations
    translation = np.array([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z,1])
    translation_current_pose =  np.array([trans_current_pose.transform.translation.x,trans_current_pose.transform.translation.y,trans_current_pose.transform.translation.z,1])
    translation_base_link = np.array([trans_base_link.transform.translation.x,trans_base_link.transform.translation.y,trans_base_link.transform.translation.z,1])
    translation_distance = np.array([trans_distance.transform.translation.x,trans_distance.transform.translation.y,trans_distance.transform.translation.z,1])

    #Compute rotations
    rotation_quaternion = pyq.Quaternion(np.array([trans.transform.rotation.w,trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z]))
    rotation_quaternion_current_pose = pyq.Quaternion(np.array([trans_current_pose.transform.rotation.w,trans_current_pose.transform.rotation.x,trans_current_pose.transform.rotation.y,trans_current_pose.transform.rotation.z]))  
    rotation_quaternion_base_link = pyq.Quaternion(np.array([trans_base_link.transform.rotation.w,trans_base_link.transform.rotation.x,trans_base_link.transform.rotation.y,trans_base_link.transform.rotation.z]))
    rotation_quaternion_distance = pyq.Quaternion(np.array([trans_distance.transform.rotation.w,trans_distance.transform.rotation.x,trans_distance.transform.rotation.y,trans_distance.transform.rotation.z]))

    #Transformation matrix
    mat = rotation_quaternion.transformation_matrix
    mat_current_pose = rotation_quaternion_current_pose.transformation_matrix
    mat_base_link = rotation_quaternion_base_link.transformation_matrix
    mat_distance =  rotation_quaternion_distance.transformation_matrix
    mat[:,3]=translation.T
    mat_current_pose[:,3]=translation_current_pose.T
    mat_base_link[:,3]=translation_base_link.T
    mat_distance[:,3]=translation_distance.T

    #Calculate distance between tag and tool
    # distance_to_tag= np.sqrt(mat_distance[0,3] + mat_distance[1,3] + mat_distance[2,3])
    # pub=rospy.Publisher("/distance_to_tag", Double, queue_size=10)
    # pub.publish(distance_to_tag)

    print("Transformation between tag and base_link is:" + str(mat))
    print("Transformation between tool0 and base_link is:" + str(mat_current_pose))
    print("Transformation between world and base_link is:" + str(mat_base_link))

    #Create an offset matrix
    Trans_goal_tag= np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.5], [0, 0, 0, 1]])
    print("Matriz de offset:" + str(Trans_goal_tag))

    #Calculate transformation matrix between world and goal and move robot to goal
    Trans_tag_goal_base=np.dot(mat, Trans_goal_tag)
    Trans_goal_base = np.eye(4)
    print(Trans_goal_base.shape)
    Trans_goal_base[:3, :3] = mat_current_pose[:3, :3]
    Trans_goal_base[:3, 3] = Trans_tag_goal_base[:3, 3]
    Trans_goal_world = np.dot(mat_base_link,Trans_tag_goal_base)
    print(Fore.GREEN + "Transformation between goal and world is:" + str(Trans_goal_world) + Style.RESET_ALL)
    move_group_obj.go_to_pose_goal(Trans_goal_world)
    print ("============ Move group completed!=============")

  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print("Transform not available")
