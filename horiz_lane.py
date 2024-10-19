import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import copy
DEBUG = 0

class lanesNode(Node):
 
    def __init__(self):
        super().__init__('lanesprocessing')
        self.bridge = CvBridge()
        self.horizflag = 0
        self.vertfinal = []
        self.cache_image = np.zeros((1200, 1200, 3), dtype = np.uint8)
        self.pcd_publisher = self.create_publisher(PointCloud2, '/igvc/sd_lanes', 10)
        self.draw_sub = self.create_subscription(Image, '/lanes_draw', self.draw_callback, 10)
        self.draw_sub

    def dist(self, x, y, p1, p2):
        d_x = (p1[0]-p2[0])
        d_y = (p1[1]-p2[1])
        c = p1[0]*d_y - p1[1]*d_x
        dist = y*d_x - x*d_y + c
        denom = d_x*d_x + d_y*d_y
        denom = denom**0.5
        return abs(dist)/denom
    
    def perpdist(self, x, y, slope, intercept):
        num = abs(slope*x - y + intercept)
        denom =(slope*2 + 1)*0.5
        return(num/denom)
    
    def get_cat(self, line):
        p1 = [line[0][0], line[0][1]]
        p2 = [line[0][2], line[0][3]]
        slope = math.atan2(p1[1]-p2[1], p1[0]-p2[0])
        if(slope < 0):
            slope += math.pi
        if(slope > math.pi):
            slope -= math.pi

        return((slope, p1[1] - math.tan(slope)*p1[0], p1, p2))
    
    def pcd(self, points, frame):
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        header = Header(frame_id=frame)

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

    def publish_lanes(self, lanes_img):
        if (DEBUG):
            cv2.imshow("lanes", lanes_img)
        binary = cv2.cvtColor(lanes_img, cv2.COLOR_BGR2GRAY)
        points = cv2.findNonZero(binary)
        lane_points = np.array([[point[0][0]/100 - 6, point[0][1]/100 - 6, 0] for point in points])
        
        # Convert lane_points to pc2 and publish on topic /igvc/sd_lanes
        pc2msg = self.pcd(lane_points, 'base_link')
        self.pcd_publisher.publish(pc2msg)

    def handle_horiz(self, vert, cdst):
        for line in vert:
            slope = line[0]
            intercept = line[1]
            y3 = int(intercept)
            y4 = int(slope*self.cache_image.shape[1] + intercept)
            if (abs(y3) > 10000):
                y3 = 10000
            if (abs(y4) > 10000):
                y4 = 10000    
            cv2.line(self.cache_image, (int(0), y3), (self.cache_image.shape[1], y4), (255,255,0), 3, cv2.LINE_AA)

        self.cache_image = cv2.cvtColor(self.cache_image, cv2.COLOR_BGR2GRAY)
        bluepoints = cv2.findNonZero(self.cache_image)
        print(bluepoints)

        # TODO: Take these points, do the inverse scaling and publish as a pointcloud

        self.horizflag = 1

    def draw_callback(self, data):

        self.img1 = self.bridge.imgmsg_to_cv2(data, "bgr8") # converting ROS image to cv image
        # cv2.imshow("original image", self.cvimage)

        #self.img1 = cv2.imread("/home/abhiyaan-nuc/gazebo_ws/testimg.png")
        
        w = self.img1.shape[1] #len(img1)
        h = self.img1.shape[0] #len(img1[0])
        cX = h//2
        cY = w//2

        M = cv2.getRotationMatrix2D((cX, cY), 0, 1.0)
        img = cv2.warpAffine(self.img1, M, (w, h), borderValue=(255,255,255))
        # cv2.imshow("abs", img)

        # finding lines with probablistic hough transform

        binaryimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(binaryimg, 50, 200, None, 3)
        cdst = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
        lanes_img = np.zeros((cdst.shape[0], cdst.shape[1], 3), dtype=np.uint8)
        lines = cv2.HoughLinesP(canny, 0.5, np.pi / 180, 10, None, 50, 10)
        categories = []

        # adding the first line to the clusters list
        line = lines[0]
        categories.append(self.get_cat(line))

        # clustering the remaining lines
        for line in lines:
            # go through all lines
            line_info = self.get_cat(line)
            slope = line_info[0]
            p1 = line_info[2]

            thresh = 10
            category_found = False
            for cat in categories:
                if(abs(slope - cat[0])<0.5 or abs(slope - math.pi - cat[0])<0.1 or abs(slope + math.pi - cat[0])<0.1) and (self.dist(p1[0], p1[1], cat[2], cat[3]) < thresh):
                    category_found = True
                    break
            if(not category_found):
                categories.append(self.get_cat(line))

        whitepoints = cv2.findNonZero(canny)
        lines2 = np.array([[math.tan(line[0]), 
                            line[1], line[2][0], 
                            line[2][1], line[3][0], 
                            line[3][1]] for line in categories])

        # checking for 45 deg
        s1 = lines2[0][0]
        edge_case = 0
        # if the slope of first line is too close to 45, 
        # usual logic doesn't work  
        if (abs(s1) - 1 < 0.1): edge_case = 1

        # figuring out which line is horizontal and which is vertical

        horiz = []
        vert = []
        for line in lines2:
            slope = line[0]
            if (not edge_case):
                # one will be more than 1
                # other will be less
                if abs(slope) < 1:
                    horiz.append(line)
                else:
                    vert.append(line)
            else:
                # one will be close to 1 and 
                # the other will be close to -1
                if (slope < 0):
                    horiz.append(line)
                else:
                    vert.append(line)   

        if (len(vert) < len(horiz)):
            # ASSUMPTION: vertical line are more
            # than horiz
            vert, horiz = horiz, vert

        newhoriz = []
        for line in horiz:
            if (line[0] - vert[0][0] > 0.3):
                newhoriz.append(line)
        horiz = newhoriz


        if (DEBUG and len(horiz)>0):
            # finding bounds of the horizontal line
            minpoint = [horiz[0][2], horiz[0][3]]
            maxpoint = [horiz[0][4], horiz[0][5]]

            if len(whitepoints) > 0 and len(horiz) > 0 and len(vert) > 0:
                mindist = 100000
                maxdist = 0
                for point in whitepoints:
                    point = point[0]
                    d1 = self.perpdist(point[0], point[1], horiz[0][0], horiz[0][1])
                    if (d1 < 10):
                        d2 = self.perpdist(point[0], point[1], vert[0][0], vert[0][1])
                        if(d2 < mindist):
                            mindist = d2
                            minpoint = [point[0], point[1]]
                        if(d2 > maxdist):
                            maxdist = d2
                            maxpoint = [point[0], point[1]]

        # drawing the lines
        if (self.horizflag == 0):
            # drawing the vertical lines
            for line in vert:
                slope = line[0]
                intercept = line[1]
                y3 = intercept
                y4 = slope*cdst.shape[1] + intercept
                if (abs(y3) > 100000000):
                    y3 = 100000000
                if (abs(y4) > 10000000):
                    y4 = 100000000    
                lane_img = cv2.line(lanes_img, (int(0), y3), (lanes_img.shape[1], y4), (255,255,0), 3, cv2.LINE_AA)
                cdst = cv2.line(cdst, (int(0), y3), (lanes_img.shape[1], y4), (255,255,0), 3, cv2.LINE_AA)

            # drawing the horizontal lines
            if(DEBUG):
                for i,line in enumerate(horiz):
                    slope = line[0]
                    intercept = line[1]
                    coords = []
                    coords.append(int((minpoint[1] - intercept)/slope))
                    coords.append(minpoint[1])
                    coords.append(int((maxpoint[1] - intercept)/slope))
                    coords.append(maxpoint[1])

                    for i in range(len(coords)):
                        point = coords[i]
                        if (abs(point) > 10000):
                            coords[i] = int(abs(point)/point*10000)
                    cv2.line(cdst, (int(coords[0]), int(coords[1])), (int(coords[2]), int(coords[3])), (0,255,255), 3, cv2.LINE_AA)
                cv2.imshow("horiz", cdst)
                cv2.waitKey(1)

            if len(horiz) > 0:
                # NOTE: need to save horiz 
                # and make sure the length is 2
                print("horiz detected")
                # The horizontal line has been detected, function to set flag to 1 and draw the image
                self.horizflag = 1
                self.cache_image = copy.deepcopy(lanes_img)
            else: self.cache_image = lanes_img

        # NOTE: need to publish in odom frame
        self.publish_lanes(self.cache_image)




def main(args=None):
    rclpy.init(args=args)
    laneNode = lanesNode()
    rclpy.spin(laneNode)
    laneNode.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
