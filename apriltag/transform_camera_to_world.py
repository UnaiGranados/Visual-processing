import pyrealsense2 as rs
import cv2
import rospy

# pipe = rs.pipeline()
# profile = pipe.start()
# try:
#   for i in range(0, 100):
#     frames = pipe.wait_for_frames()
#     for f in frames:
#       print(f.profile)
# finally:
#     pipe.stop()

# pipe = rs.pipeline()
# profile = pipe.start()

# dpt_frame = pipe.wait_for_frames().get_depth_frame().as_depth_frame()
# pixel_distance_in_meters = dpt_frame.get_distance(x,y)

# depth_stream = selection.get_stream(rs2.stream.depth).as_video_stream_profile()
# resolution = (depth_stream.width(), depth_stream.height())
# i = depth_stream.get_intrinsics()
# principal_point = (i.ppx, i.ppy)
# focal_length = (i.fx, i.fy)
# model = i.model

# #Pointcloud
# pc = rs.pointcloud()
# points = pc.calculate(depth_frame)
# pc.map_to(color_frame)

K=rospy.get_param("/camera_matrix/data")

camera_coordinates=( inv(K)*[u,v,1]* d)
world_coordinates = inv(M_ext) * [camera_coordinates, 1]

