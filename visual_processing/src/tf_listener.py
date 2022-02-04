import rospy
from tf import TransformListener

class node_tf:

    def __init__(self, *args):
        self.tf = TransformListener()
        

    def some_method(self):
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/tag_frame"):
            t = self.tf.getLatestCommonTime("/base_link", "/tag_frame")
            position, quaternion = self.tf.lookupTransform("/base_link", "/tag_frame", t)
            print (position, quaternion)

