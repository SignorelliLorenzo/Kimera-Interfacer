#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from kimera_interfacer.msg import SyncSemantic
import cv_bridge
import cv2 as cv
import os

import numpy as np
from sensor_msgs.msg import CameraInfo
import imageio
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import rospkg
from time import sleep

from label_loader import LabelLoaderAuto


def txt_to_camera_info(cam_p, img_p):

    data = np.loadtxt(cam_p)
    img = imageio.imread(img_p)

    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = img.shape[1]
    camera_info_msg.height = img.shape[0]
    camera_info_msg.K = data[:3, :3].reshape(-1).tolist()
    camera_info_msg.D = [0, 0, 0, 0, 0]
    camera_info_msg.R = data[:3, :3].reshape(-1).tolist()
    camera_info_msg.P = data[:3, :4].reshape(-1).tolist()
    camera_info_msg.distortion_model = "plumb_bob"
    return camera_info_msg


def broadcast_camera_pose(H, frames, time):

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = time
    t.header.frame_id = frames[0]
    t.child_frame_id = frames[1]
    t.transform.translation.x = H[0, 3]
    t.transform.translation.y = H[1, 3]
    t.transform.translation.z = H[2, 3]
    q = tf_conversions.transformations.quaternion_from_matrix(H)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)


def dl_mock():
    rospack = rospkg.RosPack()
    kimera_interfacer_path = rospack.get_path('kimera_interfacer')

    label_loader =LabelLoaderAuto(root_scannet= "/home/jonfrey/Datasets/scannet", confidence=0)
    
    depth_topic = rospy.get_param("~/dl_mock/depth_topic")
    image_topic = rospy.get_param("~/dl_mock/image_topic")
    seg_topic = rospy.get_param("~/dl_mock/seg_topic")

    sync_topic = rospy.get_param("~/dl_mock/sync_topic")
        
    probabilistic = rospy.get_param("~/dl_mock/probabilistic")
    

    base_link_frame = rospy.get_param("~/dl_mock/base_link_frame")
    world_frame = rospy.get_param("~/dl_mock/world_frame")

    scannet_scene_dir = rospy.get_param("~/dl_mock/scannet_scene_dir")
    label_scene_dir = rospy.get_param("~/dl_mock/label_scene_dir")


    depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)
    image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
    sem_pub = rospy.Publisher(seg_topic, Image, queue_size=1)
    sync_pub = rospy.Publisher(sync_topic, SyncSemantic, queue_size=1)
    image_cam_pub = rospy.Publisher("rgb_camera_info", CameraInfo, queue_size=1)
    depth_cam_pub = rospy.Publisher("depth_camera_info", CameraInfo, queue_size=1)

    bridge = cv_bridge.CvBridge()

    rospy.init_node('dl_mock', anonymous=True)

    rate = rospy.Rate(rospy.get_param("~/dl_mock/fps"))  # 1hz
    image_camera_info_msg = txt_to_camera_info(f"{scannet_scene_dir}/intrinsic/intrinsic_color.txt", f"{scannet_scene_dir}color/0.jpg")
    depth_camera_info_msg = txt_to_camera_info(f"{scannet_scene_dir}/intrinsic/intrinsic_depth.txt", f"{scannet_scene_dir}/color/0.jpg")
    n = 0
    seq = 0
    mapping = np.genfromtxt(f'{kimera_interfacer_path}/cfg/nyu40_segmentation_mapping.csv' , delimiter=',')
    rgb = mapping[1:, 1:4]

    per = 0 # percentage of used depth info

    frame_limit = rospy.get_param("~/dl_mock/frame_limit")
    sub = 10

    if frame_limit == -1:
        frame_limit = float('inf')
    else:
        frame_limit = int(frame_limit * sub)

    while not rospy.is_shutdown():
        n += sub
        img_p = os.path.join( scannet_scene_dir, "color", str(n)+".jpg")
        depth_p = os.path.join( scannet_scene_dir, "depth", str(n)+".png")
        label_p = os.path.join( label_scene_dir, str(n)+".png")
        if ( n > frame_limit or
            not os.path.isfile( img_p ) or
            not os.path.isfile( label_p )):
            if n < frame_limit:
                print("stopped at: ", img_p)
                print("stopped at: ", label_p)
            sleep(20)
            n = 0
            break

        time = rospy.Time.now()
        img = imageio.imread( img_p )
        depth = imageio.imread( depth_p )
        sem, _ = label_loader.get( label_p )

        print(sem.shape)
        depth[ np.random.rand( *depth.shape ) < per ] = 1

        sem_new = np.zeros( (sem.shape[0], sem.shape[1], 3) )
        for i in range(0,41):
            sem_new[sem == i, :3] = rgb[i]
        sem_new = np.uint8( sem_new )

        # publish camera pose
        H_cam = np.loadtxt(f"{scannet_scene_dir}pose/{n}.txt")
        broadcast_camera_pose( H_cam, (world_frame, base_link_frame), time)

        H, W = depth.shape[0], depth.shape[1] # 640, 1280

        # maps from image to depth
        map1, map2 = cv.initUndistortRectifyMap(
            np.array( image_camera_info_msg.K).reshape( (3,3) ) ,
            np.array([0,0,0,0]),
            np.eye(3),
            np.array( depth_camera_info_msg.K).reshape( (3,3) ),
            (640,480),
            cv.CV_32FC1)

        img = cv.remap( img,
             map1,
             map2,
             interpolation=cv.INTER_NEAREST,
             borderMode=cv.BORDER_CONSTANT,
             borderValue=0)
        
        print("SEM NEW ", sem_new.shape, sem_new.max(), sem_new.min(), probabilistic)
        sem_new = cv.remap( sem_new,
                        map1,
                        map2,
                        interpolation=cv.INTER_NEAREST,
                        borderMode=cv.BORDER_CONSTANT,
                        borderValue=0)

        mask = sem_new.sum(axis=2) == 1- rospy.get_param("~/dl_mock/ratio_reprojected")
        depth[mask] = 0
        img = bridge.cv2_to_imgmsg(img, encoding="rgb8")
        depth = bridge.cv2_to_imgmsg(depth, encoding="16UC1")
        sem_new = bridge.cv2_to_imgmsg(sem_new, encoding="rgb8" )

        img.header.frame_id = "base_link_gt"
        depth.header.frame_id = "base_link_gt"
        sem_new.header.frame_id = "base_link_gt"

        img.header.seq = seq
        depth.header.seq = seq
        sem_new.header.seq= seq

        img.header.stamp = time
        depth.header.stamp = time
        sem_new.header.stamp = time

        msg = SyncSemantic()
        msg.depth = depth
        msg.image = img
        msg.sem = sem_new
        # publish current frame
        depth_pub.publish(depth)
        image_pub.publish(img)
        sem_pub.publish(sem_new)

        sync_pub.publish(msg)

        # publish static camera info
        image_cam_pub.publish(image_camera_info_msg)
        depth_cam_pub.publish(depth_camera_info_msg)

        seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        dl_mock()
    except rospy.ROSInterruptException:
        pass