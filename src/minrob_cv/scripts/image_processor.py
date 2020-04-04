#!/usr/bin/env python

import rospy
import rospkg
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
import numpy as np

from minrob_cv.cfg import ImageProcessorConfig


class ImageProcessor():

    # Bridge instance that makes communication between OpenCV and ROS easier by
    # providing some easy to use conversions.
    bridge = CvBridge()
    background = np.zeros(shape=[720, 1280, 3], dtype=np.uint8)

    def __init__(self):
        rospy.init_node('image_processor', anonymous=True)
        # Pre-allocate a number of intermediate result publishers. Change this
        # number to get more or less publishers.
        self.num_pubs = 2
        self.pubs = []

        for i in range(self.num_pubs):
            self.pubs.append(rospy.Publisher('intermediate_image' + str(i),
                                             Image, queue_size=1))

        # Get the root of the package to read stuff from the data directory
        self.root = rospkg.RosPack().get_path('minrob_cv')
        self.logo_im = cv2.imread(self.root +
                                  '/data/LG_SAXION_GREEN_MECHS.png',
                                  cv2.IMREAD_UNCHANGED)
        self.logo_im = cv2.resize(self.logo_im, (0, 0), fx=0.1, fy=0.1,
                                  interpolation=cv2.INTER_LINEAR)

        # ------------------ ADD PUBLISH SUBJECTS HERE --------------------
        # This publisher is used to publish the final image, with edits.
        self.output = rospy.Publisher('final_image', Image, queue_size=1)

        # This publisher is used to publish images to debug
        self.pubs.append(rospy.Publisher('subtracted_background',Image, queue_size=1))


        # This subscriber gets the frames from the video (a video publisher is
        # used as in-between: video_stream_opencv package).
        self.input = rospy.Subscriber('/image_raw', Image,
                                      callback=self.image_received_callback)

        # This is a dynamic reconfiguration server, making it possible to
        # change certain parameters while the node is running.
        self.reconf = Server(ImageProcessorConfig, self.reconf_callback)
        self.config = self.reconf.config

        # This function keeps the node running and listening to calls from the
        # ROS master.
        rospy.spin()

    def image_received_callback(self, data):
        # This function is called when every time a frame is received from the
        # video. You don't have to change this.
        # The parameter 'data' contains the image as obtained from the video.
        # We use the CvBridge initialized earlier to convert this to a OpenCV
        # image type.
        im_cv2 = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # The new image is returned by the 'image_processing' function that you
        # should implement. Afterwards it is publised to the '/final_image'
        # topic.
        new_im_cv2 = self.image_processing(im_cv2, data.header.seq)
        self.output.publish(self.bridge.cv2_to_imgmsg(new_im_cv2,
                                                      encoding="bgr8"))

    def reconf_callback(self, config, level):
        # This function is called when a parameter value is reconfigured using
        # the dynamic reconfigure interface. We use it to save the complete
        # configuration to an instance variable called 'config'. You don't have
        # to change this.
        self.config = config
        return config

    def image_processing(self, im, frame_num):
        # =====================================================================
        # Here you should do the processing of the image.

        # EXAMPLE THRESHOLDING THE HSV REPRESENTATION OF THE IMAGE
        # The following example code applies a lower and upper bound threshold
        # based on HSV values. These bounds can be configured with the dynamic
        # reconfigure interface. To add parameters to configure, you can edit
        # minrob_cv/cfg/ImageProcessor.cfg and run catkin_make again.
        im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        lower_bound = (self.config.h_min,
                       self.config.s_min,
                       self.config.v_min)
        upper_bound = (self.config.h_max,
                       self.config.s_max,
                       self.config.v_max)

        im_thr = cv2.inRange(im_hsv, lower_bound, upper_bound)

        self.pubs[0].publish(self.bridge.cv2_to_imgmsg(im_thr,
                                                       encoding="passthrough"))

        # EIGEN TOEVOEGING

        im_hsv = cv2.GaussianBlur(im, (31, 31),0)

        lower_bound1 = (150,
                       50,
                       180)
        upper_bound1 = (200,
                       150,
                       220)

        mask_hsv = cv2.inRange(im_hsv, lower_bound1, upper_bound1)

        _,thresh = cv2.threshold(mask_hsv, 100, 110,0)
        M = cv2.moments(thresh)

        # Position of thomas
        cX = int(M["m10"] / M["m00"]) - int(M["m00"]*0.000033645)
        cY = int(M["m01"] / M["m00"]) - int(M["m00"]*0.000015)
        cv2.circle(im, (cX, cY), 5, (255, 255, 255), -1)

        # Find contours of billboard
        _, cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #im = cv2.drawContours(im, cnts, -1, (0,255,0), 2)

        # Load image
        thomas = cv2.imread("/home/noah/RV_CV_Assignment_2_ws/src/minrob_cv/data/RV_CV_Assignment_2_poster.jpg",
                            cv2.IMREAD_UNCHANGED)

        # Find the contour of the billboard
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 18000:
            size = cv2.minAreaRect(c)[1]
            width = int(size[0])-20
            height = int(size[1])-30          

            resized = cv2.resize(thomas, (width, height), interpolation= cv2.INTER_AREA)
            im[cY:cY+resized.shape[0], cX:cX+resized.shape[1]] = resized


        self.pubs[2].publish(self.bridge.cv2_to_imgmsg(thresh,
                                                       encoding="passthrough"))


        # EXAMPLE TO ADD THE SAXION LOGO ON TOP OF THE IMAGE
        # Treat the alpha channel as inverted mask
        _, mask_inv = cv2.threshold(self.logo_im[:, :, 3], 127, 255,
                                    cv2.THRESH_BINARY)
        # Invert the inverted mask to get the actual mask
        mask = cv2.bitwise_not(mask_inv)
        shp = mask.shape

        # Get matching portion of the original image for bitwise operators
        top_right = im[:shp[0], -shp[1]:]
        top_right_bg = cv2.bitwise_and(top_right, top_right, mask=mask)
        top_right_fg = cv2.bitwise_and(self.logo_im[:, :, :3],
                                       self.logo_im[:, :, :3], mask=mask_inv)
        top_right = cv2.add(top_right_bg, top_right_fg)

        # Write back to the original image and return
        im[:shp[0], -shp[1]:] = top_right        

        # Note that the original image is returned without any edits.
        # =====================================================================
        return im


if __name__ == "__main__":
    ip = ImageProcessor()
