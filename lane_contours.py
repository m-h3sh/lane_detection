from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
import math
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
 
class zedNODE(Node):
 
    def __init__(self):
        super().__init__('pubsub')
        self.bridge = CvBridge()
        self.THRESHOLD_LOW = 170
        self.THRESHOLD_HIGH = 255
        
        #creating publishers and subscribers
        self.final_img_publisher = self.create_publisher(Image, '/model_lanes', 1)
        self.final_img_publisher2 = self.create_publisher(Image, '/model_lanes2', 1)
        # self.camerasub = self.create_subscription(Image, '/camera1/image_raw', self.camera_callback, 10)
        self.camerasub = self.create_subscription(Image, '/camera1/image_raw', self.camera_callback, 10)
        self.camerasub
        self.camerasub2 = self.create_subscription(Image, '/short_1_camera/image_raw', self.camera_callback2, 10)
        self.camerasub2

    def quaternion_to_euler(self, x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z
    
    def getPerpCoord(self, lane, aX, aY, bX, bY, length):
        vX = bX-aX
        vY = bY-aY
        mag = math.sqrt(vX*vX + vY*vY)
        vX = vX / mag
        vY = vY / mag
        temp = vX
        vX = 0-vY
        vY = temp
        cX = bX + 2*lane*(vX * length)
        cY = bY + 2*lane*(vY * length)
        dX = aX + lane*(vX * length)
        dY = aY + lane*(vY * length)
        return int(cX), int(cY), int(dX), int(dY)
 
    def get_left_lanes(self, img):

        DIST_THRESHOLD = 800 # Distance threshold between consecutive dashes
        MASK_THRESHOLD = 3.5/8 # Fraction of the image to mask out
        PARAM_THRESHOLD = 0.4 # Difference in the (ymax-ymin/xmax-xmin) of contours (to eliminate the horizontal line)

        # img = cv2.imread("horiz.png")
        binaryimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        binaryimg = cv2.inRange(binaryimg, self.THRESHOLD_LOW, self.THRESHOLD_HIGH)

        # Pre processing the image to get better results

        mask = np.zeros(binaryimg.shape[:2], np.uint8)
        mask[int(MASK_THRESHOLD*binaryimg.shape[1]):binaryimg.shape[1], 0:binaryimg.shape[0]] = 255
        binaryimg = cv2.bitwise_and(binaryimg, binaryimg, mask=mask)
        binaryimg = cv2.blur(binaryimg, (3,3))
        binaryimg = cv2.medianBlur(binaryimg, 3)
        # cv2.imshow("threshold", binaryimg)

        # Finding contours

        blackimg = np.zeros((binaryimg.shape[0], binaryimg.shape[1], 3), dtype=np.uint8)

        contours1, hierarchy = cv2.findContours(binaryimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        newcontours = []
        contours = [contour for contour in contours1]
        solidlane = contours[0]
        firstdash = contours[0]
        thirdcontour = contours[0]

        # getting biggest contour - which corresponds to solid lane

        for contour in contours:
            if len(contour) > len(solidlane):
                solidlane = contour

        # getting the closest dashed line

        ymax = 0
        for contour in contours:
            if len(contour) < len(solidlane):
                botpoint = contour[contour[:, :, 1].argmax()][0]
                if (botpoint[1] > ymax):
                    ymax = botpoint[1]
                    firstdash = contour

        # finding ymax - ymin / xmax - xmin for the first dash

        # param = find_param(firstdash)

        # getting the remaining dashes


        dashedlines = []
        lastdash = firstdash
        for i in range(1, len(contours)):
            contour = contours[i]
            area = cv2.contourArea(contour)
            #print(area)
            if (len(contour) > 70):
                M1 = cv2.moments(contour)
                centre1x = int(M1['m10']/M1['m00'])
                centre1y = int(M1['m01']/M1['m00'])
                M2 = cv2.moments(lastdash)
                centre2x = int(M2['m10']/M2['m00'])
                centre2y = int(M2['m01']/M2['m00'])

                # param2  = find_param(contour)

                dist = ((centre2y - centre1y)**2 + (centre2x - centre1x)**2)**0.5
                # if (dist < DIST_THRESHOLD) and (abs(param2 - param) < PARAM_THRESHOLD):
                if (dist < DIST_THRESHOLD) and (area > 300):
                    dashedlines.append(contour)
                    lastdash = contour
                else:
                    newcontours.append(contour)

        # Uncomment to clear all the found contours
        #contours = newcontours

        # visualizing lanes

        blackimg = cv2.drawContours(blackimg, dashedlines, -1, (255, 255, 255), 1)
        blackimg = cv2.drawContours(blackimg, [solidlane], -1, (255, 255, 255), 1)
        # blackimg = cv2.drawContours(blackimg, [solidlane], -1, (0, 0, 255), 3)
        #cv2.imshow("contours", blackimg)

        return blackimg
    
    def get_right_lanes(self, img):
        DIST_THRESHOLD = 800 # Distance threshold between consecutive dashes
        MASK_THRESHOLD = 3.5/8 # Fraction of the image to mask out
        PARAM_THRESHOLD = 0.4 # Difference in the (ymax-ymin/xmax-xmin) of contours (to eliminate the horizontal line)

        # img = cv2.imread("horiz.png")
        binaryimg2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        binaryimg2 = cv2.inRange(binaryimg2, self.THRESHOLD_LOW, self.THRESHOLD_HIGH)

        # Pre processing the image to get better results

        mask = np.zeros(binaryimg2.shape[:2], np.uint8)
        mask[int(MASK_THRESHOLD*binaryimg2.shape[1]):binaryimg2.shape[1], 0:binaryimg2.shape[0]] = 255
        binaryimg2 = cv2.bitwise_and(binaryimg2, binaryimg2, mask=mask)
        binaryimg2 = cv2.blur(binaryimg2, (3,3))
        binaryimg2 = cv2.medianBlur(binaryimg2, 3)
        # cv2.imshow("threshold", binaryimg2)

        # Finding contours

        blackimg2 = np.zeros((binaryimg2.shape[0], binaryimg2.shape[1], 3), dtype=np.uint8)

        contours2, hierarchy = cv2.findContours(binaryimg2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        newcontours2 = []
        contours_right = [contour for contour in contours2]
        solidlane2 = contours_right[0]
        firstdash2 = contours_right[0]
        thirdcontour = contours_right[0]

        # getting biggest contour - which corresponds to solid lane

        for contour in contours_right:
            if len(contour) > len(solidlane2):
                solidlane2 = contour

        # getting the closest dashed line

        ymax2 = 0
        for contour in contours_right:
            if len(contour) < len(solidlane2):
                botpoint2 = contour[contour[:, :, 1].argmax()][0]
                if (botpoint2[1] > ymax2):
                    ymax2 = botpoint2[1]
                    firstdash2 = contour

        # finding ymax - ymin / xmax - xmin for the first dash

        # param = find_param(firstdash)

        # getting the remaining dashes


        dashedlines2 = []
        lastdash2 = firstdash2
        for i in range(1, len(contours_right)):
            contour2 = contours_right[i]
            area2 = cv2.contourArea(contour2)
            if (70 < len(contour2)):
                M1 = cv2.moments(contour2)
                centre1x2 = int(M1['m10']/M1['m00'])
                centre1y2 = int(M1['m01']/M1['m00'])
                M2 = cv2.moments(lastdash2)
                centre2x2 = int(M2['m10']/M2['m00'])
                centre2y2 = int(M2['m01']/M2['m00'])

                # param2  = find_param(contour)

                dist2 = ((centre2y2 - centre1y2)**2 + (centre2x2 - centre1x2)**2)**0.5
                # if (dist < DIST_THRESHOLD) and (abs(param2 - param) < PARAM_THRESHOLD):
                if ((dist2 < DIST_THRESHOLD) and (area > 100)):
                    dashedlines2.append(contour2)
                    lastdash2 = contour2
                else:
                    newcontours2.append(contour2)

        # Uncomment to clear all the found contours
        #contours = newcontours

        # visualizing lanes

        blackimg2 = cv2.drawContours(blackimg2, dashedlines2, -1, (255, 255, 255), 1)
        blackimg2 = cv2.drawContours(blackimg2, [solidlane2], -1, (255, 255, 255), 1)
        # blackimg = cv2.drawContours(blackimg, [solidlane], -1, (0, 0, 255), 3)
        # cv2.imshow("contours", blackimg2)

        return blackimg2

    def camera_callback(self, data):
        self.cvimage = self.bridge.imgmsg_to_cv2(data, "bgr8") # converting ROS image to cv image
        # cv2.imshow("original image", self.cvimage)
        cv2.waitKey(1)
        imgheight = self.cvimage.shape[0]
        imgwidth = self.cvimage.shape[1]
 
        binary_img = self.get_left_lanes(self.cvimage)

        imgmessage = self.bridge.cv2_to_imgmsg(binary_img, "rgb8")
        self.final_img_publisher.publish(imgmessage)

    def camera_callback2(self, data):
        self.cvimage2 = self.bridge.imgmsg_to_cv2(data, "bgr8") # converting ROS image to cv image
        # cv2.imshow("original image", self.cvimage2)
        cv2.waitKey(1)

        binary_img2 = self.get_left_lanes(self.cvimage2)

        imgmessage2 = self.bridge.cv2_to_imgmsg(binary_img2, "rgb8")
        self.final_img_publisher2.publish(imgmessage2)
 
 
def main(args=None):
    rclpy.init(args=args)
 
    velpub = zedNODE()
    rclpy.spin(velpub)
    velpub.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
