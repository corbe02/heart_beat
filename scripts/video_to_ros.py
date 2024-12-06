#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rospkg

def publish_video(video_path, topic_name, info_topic_name):
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, Image, queue_size=10)
    camera_info_pub = rospy.Publisher(info_topic_name, CameraInfo, queue_size=10)
    
    bridge = CvBridge()

    cap = cv2.VideoCapture(video_path)

    fps = cap.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)

    fx = rospy.get_param('/camera/fx', 745.0165)
    fy = rospy.get_param('/camera/fy', 745.6793)
    cx = rospy.get_param('/camera/cx', 667.0261)
    cy = rospy.get_param('/camera/cy', 366.4256)

    k1 = rospy.get_param('/camera/distortion_coefficients/k1', 0.0191)
    k2 = rospy.get_param('/camera/distortion_coefficients/k2', -0.2377)
    p1 = rospy.get_param('/camera/distortion_coefficients/p1', 0.0)
    p2 = rospy.get_param('/camera/distortion_coefficients/p2', 0.0)


    camera_info_msg = CameraInfo()
    camera_info_msg.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    camera_info_msg.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    camera_info_msg.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    camera_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
    
    camera_info_msg.D = [k1, k2, p1, p2]
    camera_info_msg.distortion_model = "plumb_bob"


    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        ros_image.header.frame_id = 'endoscope'

        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.header.frame_id = 'endoscope'

        # rospy.sleep(0.05) # slow down the video
        
        camera_info_pub.publish(camera_info_msg)
        pub.publish(ros_image)
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        rospy.init_node('video_publisher', anonymous=True)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('heart_beat_proj')
        
        # video_paths = [
        #     package_path + '/video/black_instrument_video.mp4',
        #     package_path + '/video/mask_img_video.mp4',
        #     package_path + '/video/masked_image_video.mp4',
        #     package_path + '/video/original_img_video.mp4'
        # ]
        

        original_video = rospy.get_param('~original_video','video2.1.mp4')
        video_path = original_video
        topic = '/video1/image_raw'
        info_topic = '/video1/camera_info'
        publish_video(video_path, topic, info_topic)

        """
        black_instrument_video = rospy.get_param('~black_instrument_video','black_instrument.mp4')
        mask_video = rospy.get_param('~mask_video','mask_img.mp4')
        masked_video = rospy.get_param('~masked_video','masked_img.mp4')
        original_video = rospy.get_param('~original_video','original_img.mp4')

        video_paths = [
            black_instrument_video,
            mask_video,
            masked_video,
            original_video
        ]

        topics = [
            '/video1/image_raw',
            '/video2/image_raw',
            '/video3/image_raw',
            '/video4/image_raw'
        ]

        info_topics = [
            '/video1/camera_info',
            '/video2/camera_info',
            '/video3/camera_info',
            '/video4/camera_info'
        ]

        

        from threading import Thread

        threads = []
        for video_path, topic, info_topic in zip(video_paths, topics, info_topics):
            thread = Thread(target=publish_video, args=(video_path, topic, info_topic))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()
        """

    except rospy.ROSInterruptException:
        pass
