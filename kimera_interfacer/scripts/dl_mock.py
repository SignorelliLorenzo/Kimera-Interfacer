#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from kimera_interfacer.msg import SyncSemantic
import cv_bridge
import cv2 as cv

import numpy as np
from sensor_msgs.msg import CameraInfo
import imageio
import tf_conversions
import tf2_ros
import geometry_msgs.msg


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

    depth_topic = rospy.get_param("~/dl_mock/depth_topic")
    image_topic = rospy.get_param("~/dl_mock/image_topic")
    seg_topic = rospy.get_param("~/dl_mock/seg_topic")

    sync_topic = rospy.get_param("~/dl_mock/sync_topic")


    base_link_gt_frame = rospy.get_param("~/dl_mock/base_link_gt_frame")
    base_link_frame = rospy.get_param("~/dl_mock/base_link_frame")
    world_frame = rospy.get_param("~/dl_mock/world_frame")

    depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)
    image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
    sem_pub = rospy.Publisher(seg_topic, Image, queue_size=1)
    sync_pub = rospy.Publisher(sync_topic, SyncSemantic, queue_size=1)
    image_cam_pub = rospy.Publisher("rgb_camera_info", CameraInfo, queue_size=1)
    depth_cam_pub = rospy.Publisher("depth_camera_info", CameraInfo, queue_size=1)

    bridge = cv_bridge.CvBridge()

    rospy.init_node('dl_mock', anonymous=True)
    rate = rospy.Rate(10)  # 1hz
    image_camera_info_msg = txt_to_camera_info("/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/intrinsic/intrinsic_color.txt",
                                         "/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/color/0.jpg")
    depth_camera_info_msg = txt_to_camera_info("/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/intrinsic/intrinsic_depth.txt",
                                         "/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/color/0.jpg")
    n = 0
    seq = 0
    mapping = np.genfromtxt('/home/jonas/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/cfg/nyu40_segmentation_mapping.csv' , delimiter=',')
    ids = mapping[1:, 5]
    rgb = mapping[1:, 1:4]
    while not rospy.is_shutdown():
        time = rospy.Time.now()
        print("Published Frame")

        img = imageio.imread(f"/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/color/{n}.jpg")
        depth = imageio.imread(f"/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/depth/{n}.png")
        sem = imageio.imread(f"/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/label-filt/{n}.png")

        depth[ np.random.rand( *depth.shape ) < 0.999 ] = 0

        sem_new = np.zeros( (sem.shape[0], sem.shape[1], 3) )
        for i in range(0,41):
            sem_new[sem == i, :3] = rgb[i]
        sem_new = np.uint8( sem_new )

        # publish camera pose
        H_cam = np.loadtxt(f"/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/pose/{n}.txt")
        broadcast_camera_pose( H_cam, (world_frame, base_link_frame), time)

        H, W = depth.shape[0], depth.shape[1] # 640, 1280

        depth = cv.resize(depth, dsize=(W, H), interpolation=cv.INTER_NEAREST)
        img = cv.resize(img, dsize=(W, H), interpolation=cv.INTER_CUBIC)
        sem_new = cv.resize(sem_new, dsize=(W, H), interpolation=cv.INTER_NEAREST)

        img = bridge.cv2_to_imgmsg(img, encoding="rgb8")
        depth = bridge.cv2_to_imgmsg(depth, encoding="16UC1")
        sem_new = bridge.cv2_to_imgmsg(sem_new, encoding="rgb8" ) #"rgb8")  # "mono16")

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

        n += 1
        if n > 400:
            n = 0
        seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        dl_mock()
    except rospy.ROSInterruptException:
        pass

# left_cam  -> depth
# left_cam  -> image_raw
# left_cam  -> /tesse/segmentation/image_raw
