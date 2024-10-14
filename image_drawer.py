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

lp = lg.LaserProjection()

class image_plotter(Node):
    def __init__(self):
        self.jugad = 0
        super().__init__('imageplotter')
        self.scan_subscriber = self.create_subscription(PointCloud2, '/igvc/ipm', self.ipm_callback, 100)
        self.points = []
        self.leftpoints = []
        self.rightpoints = []
        self.buffer = Buffer()
        self.bridge = CvBridge()
        self.drawer_publisher = self.create_publisher(Image, '/lanes_draw', 1)
        
        #self.camera_1_listener = self.buffer.lookup_transform('odom', self.camera_1_id, rclpy.time.Time())
        #self.camera_2_listener = self.buffer.lookup_transform('odom', self.camera_2_id, rclpy.time.Time())

        #self.tf_listener = TransformListener(self.buffer, self)

    def ipm_callback(self, pc2_msg):
        frame = pc2_msg.header.frame_id
        if self.jugad: 
            self.leftpoints = []
        else:
            self.rightpoints = []
        data = ros2_numpy.numpify(pc2_msg)
        for point in data['xyz']:
            if self.jugad:
                self.leftpoints.append([(point[0]+6)*100 , (point[1]+6)*100, 0]);
            else:
                self.rightpoints.append([(point[0]+6)*100 , (point[1]+6)*100, 0]);

        self.points = self.leftpoints
        self.points.extend(self.rightpoints)
        
        #maxx = max([i[0] for i in points])
        #maxy = max([i[1] for i in points])
        #minx = min([i[0] for i in points])
        #miny = min([i[1] for i in points])
        #img = np.zeros((int(maxy-miny+10),int(maxx-minx+10) , 3), dtype=np.uint8)
        img = np.zeros((1200, 1200, 3), dtype=np.uint8)
        for point in self.points:
            img = cv2.circle(img, (int(point[0]),int(point[1])), 1, (255, 255, 255), -1)
        # img = cv2.circle(img, (int(max([i[0] for i in points])),int(max([i[1] for i in points]))), 10, (0, 0, 255), -1)
        # img = cv2.circle(img, (int(min([i[0] for i in points])),int(min([i[1] for i in points]))), 10, (0, 0, 255), -1)
        #print("hi2")
        cv2.imshow("trial",img)
        cv2.waitKey(1)
        imgmessage = self.bridge.cv2_to_imgmsg(img, "rgb8")
        self.drawer_publisher.publish(imgmessage)

        if self.jugad: self.jugad = 0
        else: self.jugad = 1

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

