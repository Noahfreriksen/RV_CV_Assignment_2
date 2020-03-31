#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server

from minrob_cv.cfg import ImagePublisherConfig

# -----------------------------------------------------------------------------
# You don't have to edit this file.
# -----------------------------------------------------------------------------


class ImagePublisher():

    bridge = CvBridge()

    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)

        self.video_file = rospy.get_param('~video_file')

        self.pub = rospy.Publisher('image_raw', Image, queue_size=1)

        self.cap = cv2.VideoCapture(self.video_file)
        if not self.cap.isOpened():
            rospy.logerr("Unable to open video")

        self.num_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        # This is a dynamic reconfiguration server, making it possible to
        # change certain parameters while the node is running.
        self.reconf = Server(ImagePublisherConfig, self.reconf_callback)
        self.reconf_callback(self.reconf.config, 0)

        # Continue while the roscore is running and the source is opened
        while not rospy.is_shutdown() and self.cap.isOpened():

            curr_frame_num = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
            status, frame = self.cap.read()
            next_frame_num = curr_frame_num + 1

            if status:
                # Get the current frame number

                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.seq = curr_frame_num

                self.pub.publish(img_msg)

                # Set the next frame to read out
                if self.config.crop_video and \
                        (next_frame_num < self.config.start_frame or
                         next_frame_num > self.config.end_frame):
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES,
                                 self.config.start_frame)
                elif next_frame_num > self.num_frames - 1:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            else:
                rospy.logerr("Unable to read frame no. " + str(curr_frame_num))
                break

            # action
            self.rate.sleep()

        self.cap.release()

    def reconf_callback(self, config, level):
        # This function is called when a parameter value is reconfigured using
        # the dynamic reconfigure interface. We use it to save the complete
        # configuration to an instance variable called 'config'. You don't have
        # to change this.
        if config.end_frame > self.num_frames - 1:
            config.end_frame = self.num_frames - 1
        if hasattr(self, 'config'):
            if self.config.end_frame != config.end_frame:
                if config.end_frame < self.config.start_frame:
                    config.end_frame = self.config.start_frame

            if self.config.start_frame != config.start_frame:
                if config.start_frame > self.config.end_frame:
                    config.start_frame = self.config.end_frame

        self.config = config
        self.rate = rospy.Rate(config.fps)
        return config


if __name__ == "__main__":
    ip = ImagePublisher()
