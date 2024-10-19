import rclpy  # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import LaserScan, PointCloud2, Image # type: ignore
import ros2_numpy# type: ignore
import laser_geometry.laser_geometry as lg # type: ignore
import cv2
import numpy as np
from tf2_ros.buffer import Buffer 
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge
DEBUG = 0

lp = lg.LaserProjection()

class image_plotter(Node):
    def __init__(self):
        self.jugad = 0
        super().__init__('imageplotter')
        self.scan_subscriber_right = self.create_subscription(PointCloud2, '/ipm_right', self.ipm_callback_right, 100)
        self.scan_subscriber_left = self.create_subscription(PointCloud2, '/ipm_left', self.ipm_callback_left, 100)
        self.leftpoints = []
        self.rightpoints = []
        self.buffer = Buffer()
        self.bridge = CvBridge()
        self.drawer_publisher = self.create_publisher(Image, '/lanes_draw', 1)
        self.img = np.zeros((1200, 1200, 3), dtype=np.uint8)
        self.rightimg = np.zeros((1200, 1200, 3), dtype=np.uint8)
        self.leftimg = np.zeros((1200, 1200, 3), dtype=np.uint8)
        
        #self.camera_1_listener = self.buffer.lookup_transform('odom', self.camera_1_id, rclpy.time.Time())
        #self.camera_2_listener = self.buffer.lookup_transform('odom', self.camera_2_id, rclpy.time.Time())

        #self.tf_listener = TransformListener(self.buffer, self)

    def ipm_callback_right(self, pc2_msg):
        self.rightimg = np.zeros((1200, 1200, 3), dtype=np.uint8)
        self.rightpoints = []
        data = ros2_numpy.numpify(pc2_msg)
        for point in data['xyz']:
            self.rightpoints.append([(point[0]+6)*100 , (point[1]+6)*100, 0])
        for point in self.rightpoints:
            self.rightimg = cv2.circle(self.rightimg, (int(point[0]),int(point[1])), 1, (255, 255, 255), -1)

    def ipm_callback_left(self, pc2_msg1):
        self.leftimg = np.zeros((1200, 1200, 3), dtype=np.uint8)
        self.leftpoints = []
        data1 = ros2_numpy.numpify(pc2_msg1)
        for point1 in data1['xyz']:
            self.leftpoints.append([(point1[0]+6)*100 , (point1[1]+6)*100, 0])
        for point1 in self.leftpoints:
            self.leftimg = cv2.circle(self.leftimg, (int(point1[0]),int(point1[1])), 1, (255, 255, 255), -1)

        # Publishing the image
        self.img = cv2.bitwise_or(self.leftimg, self.rightimg)
        imgmessage = self.bridge.cv2_to_imgmsg(self.img, "rgb8")
        self.drawer_publisher.publish(imgmessage)

        if (DEBUG):
            cv2.imshow("ipm data",self.img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    # nav = BasicNavigator()

    # pmsg = PoseStamped()
    # pmsg.header.stamp = nav.get_clock().now().to_msg()
    # pmsg.header.frame_id = 'odom'
    # nav.setInitialPose()

    minimal_publisher = image_plotter()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

