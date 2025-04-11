#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from kimera_interfacer.msg import SyncSemantic
from PILBridge import PILBridge
import cv2 as cv
import os

import numpy as np
from sensor_msgs.msg import CameraInfo
import imageio.v2 as imageio
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import rospkg
from time import sleep

from label_loader import LabelLoaderAuto

from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import time
import torch
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning, module="numpy")
warnings.filterwarnings("ignore")

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


def broadcast_camera_pose(H, frames, stamp):

  br = tf2_ros.TransformBroadcaster()
  t = geometry_msgs.msg.TransformStamped()

  t.header.stamp = stamp
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
  kimera_interfacer_path = rospack.get_path("kimera_interfacer")
  label_loader = LabelLoaderAuto(
    root_scannet=rospy.get_param("~/dl_mock/root_scannet"),
    confidence=rospy.get_param("~/dl_mock/prob_main"),
  )

  label_loader_aux = LabelLoaderAuto(
    root_scannet=rospy.get_param("~/dl_mock/root_scannet"),
    confidence=rospy.get_param("~/dl_mock/prob_aux"),
  )

  depth_topic = rospy.get_param("~/dl_mock/depth_topic")
  image_topic = rospy.get_param("~/dl_mock/image_topic")
  seg_topic = rospy.get_param("~/dl_mock/seg_topic")

  sync_topic = rospy.get_param("~/dl_mock/sync_topic")

  probabilistic = rospy.get_param("~/dl_mock/probabilistic")

  base_link_frame = rospy.get_param("~/dl_mock/base_link_frame")
  world_frame = rospy.get_param("~/dl_mock/world_frame")

  scannet_scene_dir = rospy.get_param("~/dl_mock/scannet_scene_dir")
  label_scene_dir = rospy.get_param("~/dl_mock/label_scene_dir")
  print("Launching dl_mock node...")

  # Confirm parameters
  print("DL_MOCK Params:")
  print(f"Scene dir: {scannet_scene_dir}")
  print(f"Label dir: {label_scene_dir}")
  print(f"Image topic: {image_topic}")
  print(f"Depth topic: {depth_topic}")
  print(f"Seg topic: {seg_topic}")
  print(f"Sync topic: {sync_topic}")
  assert os.path.isdir(scannet_scene_dir), f"Scene directory not found: {scannet_scene_dir}"
  assert os.path.isdir(label_scene_dir), f"Label directory not found: {label_scene_dir}"
  depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)
  image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
  sem_pub = rospy.Publisher(seg_topic, Image, queue_size=1)
  sync_pub = rospy.Publisher(sync_topic, SyncSemantic, queue_size=1)
  image_cam_pub = rospy.Publisher("rgb_camera_info", CameraInfo, queue_size=1)
  depth_cam_pub = rospy.Publisher("depth_camera_info", CameraInfo, queue_size=1)

  #bridge = cv_bridge.CvBridge()

  rospy.init_node("dl_mock", anonymous=True)

  rate = rospy.Rate(rospy.get_param("~/dl_mock/fps"))  # 1hz
  image_camera_info_msg = txt_to_camera_info(
    f"{scannet_scene_dir}/intrinsic/intrinsic_color.txt",
    f"{scannet_scene_dir}/color/0.jpg",
  )
  depth_camera_info_msg = txt_to_camera_info(
    f"{scannet_scene_dir}/intrinsic/intrinsic_depth.txt",
    f"{scannet_scene_dir}/color/0.jpg",
  )
  n = 0
  seq = 0
  mapping = np.genfromtxt(
    f"{kimera_interfacer_path}/cfg/nyu40_segmentation_mapping.csv", delimiter=","
  )
  rgb = mapping[1:, 1:4]

  per = 0  # percentage of used depth info

  frame_limit = rospy.get_param("~/dl_mock/frame_limit")
  sub_reprojected = rospy.get_param("~/dl_mock/sub_reprojected")
  if frame_limit == -1:
    frame_limit = float("inf")
  else:
    frame_limit = int(frame_limit)

  from pathlib import Path

  label_paths = [str(s) for s in Path(label_scene_dir).rglob("*.png")]
  label_paths = [l for l in label_paths if l.find("_.png") == -1]
  label_paths = [l for l in label_paths if int(l.split("/")[-1][:-4]) % 10 == 0]

  label_paths.sort(
    key=lambda x: 1000000 * int(str(x).split("/")[-3][5:9])
    + 10000 * int(str(x).split("/")[-3][-2:])
    + int(x.split("/")[-1][:-4])
  )

  depth_paths = [
    os.path.join(scannet_scene_dir, "depth", s.split("/")[-1]) for s in label_paths
  ]
  image_paths = [
    os.path.join(scannet_scene_dir, "color", s.split("/")[-1][:-4] + ".jpg")
    for s in label_paths
  ]

  aux = [False for i in label_paths]

  aux_labels = [rospy.get_param("~/dl_mock/aux_labels")]
  if len(aux_labels) != 0 and os.path.isdir(aux_labels[0]):
    new_paths = []
    new_depths = []
    new_image_paths = []
    aux = []
    for k, p in enumerate(label_paths):
      aux.append(False)
      new_paths.append(p)
      new_depths.append(depth_paths[k])
      new_image_paths.append(image_paths[k])
      for j in range(len(aux_labels)):
        aux.append(True)
        new_depths.append(depth_paths[k])
        new_image_paths.append(image_paths[k])
        new_paths.append(os.path.join(aux_labels[j], p.split("/")[-1]))

    depth_paths = new_depths
    label_paths = new_paths
    image_paths = new_image_paths

  integrated = 0

  class Test(Dataset):
    def __init__(self, aux, label_paths, depth_paths, image_paths, sub_reprojected):
      self.aux, self.label_paths, self.depth_paths, self.image_paths = (
        aux,
        label_paths,
        depth_paths,
        image_paths,
      )
      self.length = len(self.aux)
      for j, a in enumerate(zip(label_paths, depth_paths)):
        l, d = a
        if not os.path.isfile(d) or not os.path.isfile(l):
          self.length = j
      self.map1, self.map2 = cv.initUndistortRectifyMap(
        np.array(image_camera_info_msg.K).reshape((3, 3)),
        np.array([0, 0, 0, 0]),
        np.eye(3),
        np.array(depth_camera_info_msg.K).reshape((3, 3)),
        (640, 480),
        cv.CV_32FC1,
      )
      self.sub_reprojected = sub_reprojected

    def __getitem__(self, index):

      aux_flag, label_p, depth_p, image_p = (
        self.aux[index],
        self.label_paths[index],
        self.depth_paths[index],
        self.image_paths[index],
      )
            
      

      assert os.path.isfile(depth_p), f"Missing depth file: {depth_p}"
      assert os.path.isfile(image_p), f"Missing image file: {image_p}"
      assert os.path.isfile(label_p), f"Missing label file: {label_p}"
      depth = imageio.imread(depth_p)
      mask = np.zeros_like(np.array(depth))
      mask[::sub_reprojected, ::sub_reprojected] = 1
      depth[mask != 1] = 0
      
      img = imageio.imread(image_p)
      
      if aux_flag:
        sem, _ = label_loader_aux.get(label_p)
      else:
        sem, _ = label_loader.get(label_p)

      sem_new = np.zeros((sem.shape[0], sem.shape[1], 3))
      for i in range(0, 41):
        sem_new[sem == i, :3] = np.copy(rgb[i])
      sem_new = np.uint8(sem_new)

      # publish camera pose
      n = int(label_p.split("/")[-1][:-4])
      H_cam = np.loadtxt(f"{scannet_scene_dir}/pose/{n}.txt")

      H, W = depth.shape[0], depth.shape[1]  # 640, 1280

      # maps from image to depth
      img = cv.remap(
        img,
        self.map1,
        self.map2,
        interpolation=cv.INTER_NEAREST,
        borderMode=cv.BORDER_CONSTANT,
        borderValue=0,
      )

      sem_new = cv.remap(
        sem_new,
        self.map1,
        self.map2,
        interpolation=cv.INTER_NEAREST,
        borderMode=cv.BORDER_CONSTANT,
        borderValue=0,
      )

      return (np.array(img), np.array(depth.astype(np.int32)), np.array(sem_new), H_cam)

    def __len__(self):

      return self.length

  data = Test(
    aux, label_paths, depth_paths, image_paths, sub_reprojected=sub_reprojected
  )
  dataloader = torch.utils.data.DataLoader(data, num_workers=8, shuffle=False, timeout=10)
  iterator = iter(dataloader)
  for j, batch in enumerate(dataloader):
    load_start = time.time()
    try:
        batch = next(iterator)
    except StopIteration:
        break
    load_time = time.time() - load_start

    process_start = time.time()
    if j == len(dataloader) - 1:
      print("Break because of last frame !")
      break

    if j > frame_limit:
      print("Break because of frame limit !")
      break

    if rospy.is_shutdown():
      print("Break because rospy shutdown !")
      break

    print(j, "/", len(dataloader))
    img, depth, sem_new, H_cam = (
      batch[0].numpy()[0],
      batch[1].numpy().astype(np.uint16)[0],
      batch[2].numpy()[0],
      batch[3].numpy()[0],
    )

    t = rospy.Time.now()

    img = PILBridge.numpy_to_rosimg(img, encoding="rgb8")
    depth = PILBridge.numpy_to_rosimg(depth, encoding="16UC1")
    sem_new = PILBridge.numpy_to_rosimg(sem_new, encoding="rgb8")

    for msg in (img, depth, sem_new):
      msg.header.frame_id = "base_link_gt"
      msg.header.seq = j
      msg.header.stamp = t

    msg = SyncSemantic()
    msg.depth = depth
    msg.image = img
    msg.sem = sem_new

    broadcast_camera_pose(H_cam, (world_frame, base_link_frame), t)

    # publish current frame
    depth_pub.publish(depth)
    image_pub.publish(img)
    sem_pub.publish(sem_new)
    sync_pub.publish(msg)

    # publish static camera info
    image_cam_pub.publish(image_camera_info_msg)
    depth_cam_pub.publish(depth_camera_info_msg)

    process_time = time.time() - process_start
    total_time = load_time + process_time

    print(f"[{j}] Published frame at {t.to_sec():.2f}")
    print(f"    Load Time:    {load_time:.3f}s")
    print(f"    Process Time: {process_time:.3f}s")
    rate.sleep()

  print("Start Sleeping for 50s \n" * 5)
  time.sleep(50)
  print("Finished Sleeping for 50s \n" * 5)


if __name__ == "__main__":
  try:
    dl_mock()
  except rospy.ROSInterruptException:
    pass
