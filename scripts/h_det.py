#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3Stamped



####################################################################
class ShapeDetector:
    def __init__(self):
        rospy.init_node("h_detector")
        self.rate = rospy.Rate(60)
        self.detection = Vector3Stamped()
        self.font = cv2.FONT_HERSHEY_COMPLEX
        self.detection_pub = rospy.Publisher("/cv_detection/detection", Vector3Stamped, queue_size=1)
        self.bridge = CvBridge()
        self.gray = np.zeros((256, 256, 1), dtype = "uint8")
        self.image_sub = rospy.Subscriber("/tello/image_raw", Image, self.image_callback)

    def image_callback(self, image):
        try:
            self.gray = self.bridge.imgmsg_to_cv2(image, "mono8")
            self.detect()
            # self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            # self.gray = self.cv_image

        except CvBridgeError as e:
            print ("CvBridge Error: " + str(e))

    def detect(self):
        # Capture frame-by-frame
            blur1 = cv2.GaussianBlur(self.gray, (9,9), 0)
            blur2 = cv2.GaussianBlur(blur1, (9,9), 0)
            _, thresh = cv2.threshold(blur2, 120, 255, cv2.THRESH_BINARY)
            _, cnts, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # kernel = np.array([[0, 0, 255, 255, 0, 0],
            #                    [0, 0, 255, 255, 0, 0],
            #                    [0, 0, 0, 0, 0, 0],
            #                    [0, 0, 0, 0, 0, 0],
            #                    [0, 0, 255, 255, 0, 0],
            #                    [0, 0, 255, 255, 0, 0]], np.int32)
            kernel = np.array([[0, 255, 0],
                            [0, 0,   0],
                            [0, 255, 0]], np.float64)
            cv2.imshow("Convolution kernel", kernel)

            for cnt in cnts:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
                #cv2.drawContours(self.gray, [approx], 0, (0), 2)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                
                #if len(approx) == 3:
                    #cv2.putText(self.gray, "Triangulo", (x,y), self.font, 1, (0))
                    #cv2.drawContours(self.gray, [approx], 0, (0), 2)
                if len(approx) == 12:
                    cv2.putText(self.gray, "H", (x,y), self.font, 1, (0, 255, 0))
                    cv2.drawContours(self.gray, [approx], 0, (0, 255, 0), 2)
                    M = cv2.moments(cnt)
                    self.detection.vector.x = int(M["m10"] / M["m00"])
                    self.detection.vector.y = int(M["m01"] / M["m00"])
                    self.detection.vector.z = cv2.contourArea(cnt)/(self.gray.shape[0]*self.gray.shape[1])
                    self.detection_pub.publish(self.detection)

                    box = np.float32([[0, 0],
                                    [thresh.shape[0], 0],
                                    [thresh.shape[0], thresh.shape[1]],
                                    [0, thresh.shape[1]]])
                    
                    self.edge_pts = np.float32([
                                        [approx[0][0][0], approx[0][0][1]],
                                        [approx[11][0][0], approx[11][0][1]],
                                        [approx[5][0][0], approx[5][0][1]],
                                        [approx[6][0][0], approx[6][0][1]] ])
                    transformed = self.four_point_transform(thresh, self.edge_pts)
                    cv2.imshow("Transformed Image", transformed)
                    # M = cv2.getPerspectiveTransform(self.edge_pts, box)
                    # small_img = cv2.warpPerspective(thresh, M, thresh.shape)

                    small_img = cv2.resize(transformed, (3, 3), interpolation=cv2.INTER_AREA)
                    cv2.imshow("Small Image", small_img)
                    result = small_img.dot(kernel)
                    # i=0
                    # for v in approx:
                    #     cv2.putText(self.gray, str(i), (v[0][0], v[0][1]), self.font, 1, (0, 255, 0))
                    #     i += 1

            # Our operations on the self.gray come here
            #gray = cv2.cvtColor(self.gray, cv2.COLOR_BGR2GRAY)

            # Display the resulting self.gray
            cv2.imshow('frame', self.gray)

    def order_points(self, pts):
        # initialzie a list of coordinates that will be ordered
        # such that the first entry in the list is the top-left,
        # the second entry is the top-right, the third is the
        # bottom-right, and the fourth is the bottom-left
        rect = np.zeros((4, 2), dtype = "float32")
        # the top-left point will have the smallest sum, whereas
        # the bottom-right point will have the largest sum
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        # now, compute the difference between the points, the
        # top-right point will have the smallest difference,
        # whereas the bottom-left will have the largest difference
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        # return the ordered coordinates
        return rect

    def four_point_transform(self, image, pts):
        # obtain a consistent order of the points and unpack them
        # individually
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")
        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        # return the warped image
        return warped

####################################################################
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

if __name__ == "__main__" :

    sd = ShapeDetector()
    sd.run()