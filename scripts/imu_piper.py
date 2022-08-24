import math

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


from copy import deepcopy




class Nodo(object):
    def __init__(self):
        # Params
        #self.image = None
        #self.br = CvBridge()
        # Node cycle rate (in Hz).
        #self.loop_rate = rospy.Rate(1)

        

        self.imuPub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)
        self.odomPub = rospy.Publisher('/body_pose_ground_truth_rotation_only', Odometry, queue_size=1)
        rospy.init_node('Helper_Imu_Piper_node', anonymous=True)

        self.imuSub=rospy.Subscriber('/gazebo_ros_imu', Imu,self.imucallback,queue_size=1)
        self.odomSub=rospy.Subscriber('/body_pose_ground_truth', Odometry,self.poscallback,queue_size=1)
        #rospy.Subscriber("/camera/image_color",Image,self.callback)
        #Data Memory

        self.Imu_=Imu()
        self.PoseStamped_=PoseStamped()
        rospy.spin()
#    def callback(self, msg):
#        rospy.loginfo('Image received...')
#        self.image = self.br.imgmsg_to_cv2(msg)


    def imucallback(self,data):
        #self.Imu_=deepcopy(data)
        data.header.frame_id="map"
        #data.linear_acceleration.z=data.linear_acceleration.z+9.81

        self.imuPub.publish(data)
    def poscallback(self,data):
        #self.PoseStamped_=deepcopy(data)
        data.pose.pose.position.x=0
        data.pose.pose.position.y=0
        data.pose.pose.position.z=0
        self.odomPub.publish(data)

if __name__ == '__main__':
    my_node = Nodo()

    #my_node.start()
