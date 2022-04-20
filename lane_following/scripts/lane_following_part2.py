#!/usr/bin/env python

from cv2 import mean
from matplotlib.pyplot import thetagrids
import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

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

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                h, w, d = image.shape
                bev = cv2.warpPerspective(image, self.homography, (w,h))

                hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)

                lower_yellow = numpy.array([26, 43, 46])
                upper_yellow = numpy.array([34, 255, 255])
                lower_white = numpy.array([0, 0, 221])
                upper_white = numpy.array([180, 30, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

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
                
                gray = mask1 + mask2
                curv = self.cal_curvature(gray)
                curv = max(0, numpy.mean(curv) - 0.15075)

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0 and M2['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2

                    cv2.circle(bev, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(bev, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(bev, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
                    theta = math.atan2(err, fpt_y)

                    self.twist.angular.z =   5 * theta + 0.5 * (err*90.0/160)/15
                    if err != 0:
                            self.twist.linear.x = 0.5 - 2*curv
                    else:
                            self.twist.linear.x = 0.5
                    #print(4*theta,  (err*90.0/160)/15, self.twist.linear.x)
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
