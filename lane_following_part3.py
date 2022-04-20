#!/usr/bin/env python

from __future__ import print_function
from distutils.log import INFO
import rospy, cv2, cv_bridge, numpy, time, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.ARUCO_DICT = {
                         "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
                        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
                        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
                         "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
                         "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
                         "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
                         "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
                        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
                         "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
                        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
                        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
                        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
                         "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
                         "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
                         "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
                         "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
                        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
                }

                self.D = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0])

                self.K = numpy.array([[-3.313061098815712, 0.0, 160.5], [0.0, -3.313061098815712, 120.5],[0.0, 0.0, 1.0]])

                self.isStop = False

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                pts_src = numpy.array([[25, 239], [288, 239], [135, 141],[183, 141]])
                pts_dst = numpy.array([[110, 240],[210, 240],[110, 120],[210, 120]])
                self.homography, self.status = cv2.findHomography(pts_src, pts_dst)

        def image_callback(self, msg):

                this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_6X6_250"])
                this_aruco_parameters = cv2.aruco.DetectorParameters_create()

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                h, w, d = image.shape
                bev = cv2.warpPerspective(image, self.homography, (w,h))
                hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)

                (corners, ids, rejected) = cv2.aruco.detectMarkers(image, this_aruco_dictionary, parameters=this_aruco_parameters)

                lower_yellow = numpy.array([26, 43, 46])
                upper_yellow = numpy.array([34, 255, 255])

                lower_white = numpy.array([0, 0, 221])
                upper_white = numpy.array([180, 30, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)
                mask3 = cv2.inRange(hsv, lower_white, upper_white)

                search_top = 140
                search_bot = 200
                search_left = 60
                search_right = 260
                mask1[0:search_top, 0:w] = 0
                mask1[0:h, 0:search_left] = 0
                mask1[0:h, search_right:w] = 0
                mask1[search_bot:h, 0:w] = 0
                mask2[0:search_top, 0:w] = 0
                mask2[0:h, 0:search_left] = 0
                mask2[0:h, search_right:w] = 0
                mask2[search_bot:h, 0:w] = 0
                mask3[0:search_top, 0:w] = 0
                mask3[0:h, 0:search_left] = 0
                mask3[0:h, 230:w] = 0
                mask3[search_bot:h, 0:w] = 0

                gray = mask1 + mask2
                curv = self.cal_curvature(gray)
                curv = max(0, numpy.mean(curv) - 0.15075)

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)
                M3 = cv2.moments(mask3)

                if len(corners) > 0:
                  if M1['m00'] > 0 and M3['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M3['m10']/M3['m00'])
                    cy2 = int(M3['m01']/M3['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(bev, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(bev, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    theta = math.atan2(err, fpt_y)
                
                    ids = ids.flatten()
                    for (marker_corner, marker_id) in zip(corners, ids):
                      marker_corners = marker_corner.reshape((4, 2))
                      (top_left, top_right, bottom_right, bottom_left) = marker_corners
                      top_right = (int(top_right[0]), int(top_right[1]))
                      bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                      bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                      top_left = (int(top_left[0]), int(top_left[1]))
                      cv2.line(image, top_left, top_right, (0, 255, 0), 2)
                      cv2.line(image, top_right, bottom_right, (0, 255, 0), 2)
                      cv2.line(image, bottom_right, bottom_left, (0, 255, 0), 2)
                      cv2.line(image, bottom_left, top_left, (0, 255, 0), 2)
                      cv2.putText(image, str(marker_id), (top_left[0], top_left[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                  
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.33, self.K, self.D)
                    if tvec[:,:,2] > 0.02:
                      print("[INFO] Stop Tag Detected! ID: ArUco DICT_6X6_250"+str(ids)+" Distance: "+str(tvec[:,:,2]))

                    if tvec[:,:,2] < 0.09 and tvec[:,:,2] > 0.08 and self.isStop == False:
                      self.twist.linear.x = 0
                      self.twist.angular.z = 0
                      print("[INFO] Stop Time! Distance: "+str(tvec[:,:,2]))
                      self.cmd_vel_pub.publish(self.twist)
                      self.isStop = True
                      time.sleep(10)
                      print("[INFO] Move On!")
                      self.twist.linear.x = 0.3
                      self.twist.angular.z = (err*90.0/160)/15
                      self.cmd_vel_pub.publish(self.twist)

                    else:
                          self.twist.linear.x = 0.3
                          self.twist.angular.z = (err*90.0/160)/15
                          if tvec[:,:,2] < 0.08:
                            self.isStop = False
                          self.cmd_vel_pub.publish(self.twist)
                
                else:
                  self.isStop = False
                  if M1['m00'] > 0 and M2['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(bev, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(bev, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    theta = math.atan2(err, fpt_y)
                    self.twist.angular.z = 3 * theta + 0.5 * (err*90.0/160)/15
                    if err != 0:
                            self.twist.linear.x = 0.45 - 2*curv
                    else:
                            self.twist.linear.x = 0.45
                    self.cmd_vel_pub.publish(self.twist)

                cv2.imshow("camera view", image)
                cv2.imshow("bird eye view",bev)
                cv2.waitKey(1)

        def cal_curvature(self, img):
                
                x , y = numpy.gradient(img)
                xx, xy = numpy.gradient(x)
                yx, yy = numpy.gradient(y)
                Iup =  (1+x*x)*yy - 2*x*y*xy + (1+y*y)*xx
                Idown = 2*numpy.power((1 + x*x + y*y),1.5)
                curv  = Iup/Idown
                curv = abs(curv)
                return curv

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
