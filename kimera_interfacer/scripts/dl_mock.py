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
import rospkg
from time import sleep
import pandas
import torch

def load_label_scannet( p, mapping_scannet):
    label_gt = imageio.imread( p )
    label_gt = torch.from_numpy(label_gt.astype(np.int32)).type(
        torch.float32)[:, :]  # H W
    sa = label_gt.shape
    label_gt = label_gt.flatten()
    print(p)

    label_gt = mapping_scannet[label_gt.type(torch.int64)]
    label_gt = label_gt.reshape(sa) # 1 == chairs 40 other prop  0 invalid

    return label_gt.numpy()

def load_label_detectron( p, ignore):
    label_gt = imageio.imread( p )
    return label_gt


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

def get_mapping_scannet(p):
    df = pandas.read_csv(p, sep='\t')
    mapping_source = np.array( df['id'] )
    mapping_target = np.array( df['nyu40id'] )
    mapping_scannet = torch.zeros( ( int(mapping_source.max()+1) ),dtype=torch.int64)
    for so,ta in zip(mapping_source, mapping_target):
        mapping_scannet[so] = ta
    return mapping_scannet


def dl_mock():
    mapping_scannet = get_mapping_scannet("/home/jonfrey/Datasets/scannet/scannetv2-labels.combined.tsv")
    rospack = rospkg.RosPack()
    kimera_interfacer_path = rospack.get_path('kimera_interfacer')

    depth_topic = rospy.get_param("~/dl_mock/depth_topic")
    image_topic = rospy.get_param("~/dl_mock/image_topic")
    seg_topic = rospy.get_param("~/dl_mock/seg_topic")

    sync_topic = rospy.get_param("~/dl_mock/sync_topic")
    seg_folder = rospy.get_param("~/dl_mock/seg_folder")

    if seg_folder == "label-filt":
        load_label = load_label_scannet
    else:
        load_label = load_label_detectron

    base_link_gt_frame = rospy.get_param("~/dl_mock/base_link_gt_frame")
    base_link_frame = rospy.get_param("~/dl_mock/base_link_frame")
    world_frame = rospy.get_param("~/dl_mock/world_frame")

    base_path = "/home/jonfrey/Datasets/scannet_scene_0000/" # rospy.get_param("~/dl_mock/base_path")

    depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)
    image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
    sem_pub = rospy.Publisher(seg_topic, Image, queue_size=1)
    sync_pub = rospy.Publisher(sync_topic, SyncSemantic, queue_size=1)
    image_cam_pub = rospy.Publisher("rgb_camera_info", CameraInfo, queue_size=1)
    depth_cam_pub = rospy.Publisher("depth_camera_info", CameraInfo, queue_size=1)

    bridge = cv_bridge.CvBridge()

    rospy.init_node('dl_mock', anonymous=True)

    rate = rospy.Rate(3)  # 1hz
    image_camera_info_msg = txt_to_camera_info(f"{base_path}/intrinsic/intrinsic_color.txt", f"{base_path}color/0.jpg")
    depth_camera_info_msg = txt_to_camera_info(f"{base_path}/intrinsic/intrinsic_depth.txt", f"{base_path}/color/0.jpg")
    n = 0
    seq = 0
    mapping = np.genfromtxt(f'{kimera_interfacer_path}/cfg/nyu40_segmentation_mapping.csv' , delimiter=',')
    ids = mapping[1:, 5]
    rgb = mapping[1:, 1:4]

    per = 0 # percentage of used depth info
    while not rospy.is_shutdown():
        n += 10
        if n > 5570: #5570: #5570:
            # stops the kimera interfacer to gernerate mesh
            sleep(20)
            n = 0
            # break
        time = rospy.Time.now()
        img = imageio.imread(f"{base_path}/color/{n}.jpg")
        depth = imageio.imread(f"{base_path}/depth/{n}.png")
        sem = load_label( f"{base_path}/{seg_folder }/{n}.png", mapping_scannet)

        depth[ np.random.rand( *depth.shape ) < per ] = 1

        sem_new = np.zeros( (sem.shape[0], sem.shape[1], 3) )
        for i in range(0,41):
            sem_new[sem == i, :3] = rgb[i]
        sem_new = np.uint8( sem_new )

        # publish camera pose
        H_cam = np.loadtxt(f"{base_path}pose/{n}.txt")
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
        sem_new = cv.remap( sem_new,
                        map1,
                        map2,
                        interpolation=cv.INTER_NEAREST,
                        borderMode=cv.BORDER_CONSTANT,
                        borderValue=0)
        # from PIL import Image as ImagePIL
        # img2 = ImagePIL.fromarray(np.uint8(np.concatenate( [img,sem_new] )))
        # img2.show()

        mask = sem_new.sum(axis=2) == 0
        depth[mask] = 0
        #
        # depth = cv.resize(depth, dsize=(W, H), interpolation=cv.INTER_NEAREST)
        # img = cv.resize(img, dsize=(W, H), interpolation=cv.INTER_CUBIC)
        # sem_new = cv.resize(sem_new, dsize=(W, H), interpolation=cv.INTER_NEAREST)

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
