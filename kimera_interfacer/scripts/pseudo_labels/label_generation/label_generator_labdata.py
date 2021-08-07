if __name__ == "__main__":
  import os
  import sys

  sys.path.append(os.path.dirname(os.getcwd()))

try:
  from label_generation import VoxelMap
  from label_generation import RayCaster
  from label_generation import Visualizer3D
except:
  from . import VoxelMap
  from . import RayCaster
  from . import Visualizer3D

from ssl import RAND_bytes
import numpy as np
import imageio
import os


class LabelGenerator:
  def __init__(self, arg, visu3d=False):
    # Pass the args config

    r_sub = arg.r_sub
    mesh_path = arg.mesh_path
    map_serialized_path = arg.map_serialized_path
    scannet_scene_dir = arg.scannet_scene_dir
    # GT LABEL TEST
    label_gt = imageio.imread(os.path.join(scannet_scene_dir, "label-filt/0" + ".png"))
    H, W, _ = label_gt.shape
    size = (H, W)

    data = np.loadtxt(f"{scannet_scene_dir}/intrinsic/intrinsic_color.txt")
    k_image = np.array(
      [[data[0, 0], 0, data[0, 2]], [0, data[1, 1], data[1, 2]], [0, 0, 1]]
    )

    self.r_sub = r_sub
    self._voxel_map = VoxelMap(map_serialized_path, size, r_sub)
    self._ray_caster = RayCaster(mesh_path, k_image, size, r_sub)

    self._visu_active = visu3d
    if visu3d == True:
      self._visu3d = Visualizer3D(size, k_image, mesh_path)

  def get_label(self, H_cam):
    locations, index_ray, index_tri, ray_origins = self._ray_caster.raycast(H_cam)

    probs = self._voxel_map.ray_cast_results_to_probs(locations, index_ray)
    if self._visu_active:
      self._visu3d.visu(locations, ray_origins)

    return probs


import imageio

import time


def label_to_png2(label, path):
  max_classes = 40
  H, W, _ = label.shape
  idxs = np.zeros((3, H, W), dtype=np.uint8)
  values = np.zeros((3, H, W))
  label_c = label  # .clone()
  max_val_10bit = 1023

  for i in range(3):
    idx = np.argmax(label_c, axis=2)
    idxs[i] = idx.astype(np.int32)

    m = np.eye(max_classes)[idx] == 1
    values[i] = ((label_c[m] * max_val_10bit).reshape(H, W)).astype(np.int32)
    values[i][values[i] > max_val_10bit] = max_val_10bit
    label_c[m] = -0.1

  values = values.astype(np.int32)
  idxs = idxs.astype(np.int32)

  png = np.zeros((H, W, 4), dtype=np.int32)
  for i in range(3):
    png[:, :, i] = values[i]
    png[:, :, i] = np.bitwise_or(png[:, :, i], idxs[i] << 10)
  imageio.imwrite(path, png.astype(np.uint16), format="PNG-FI", compression=9)

  print("Done")
  return True


if __name__ == "__main__":
  import argparse

  parser = argparse.ArgumentParser()
  # EXTERNAL DATA PATHS
  parser.add_argument(
    "--scannet_scene_dir",
    type=str,
    default="/home/jonfrey/Datasets/labdata/result/full_run_2",
    help="",
  )
  parser.add_argument(
    "--mesh_path",
    type=str,
    default="/home/jonfrey/Datasets/labdata/result/full_run_2/full_run_2_predict_mesh.ply",
    help="",
  )
  parser.add_argument(
    "--map_serialized_path",
    type=str,
    default="/home/jonfrey/Datasets/labdata/result/full_run_2/full_run_2_serialized.data",
    help="",
  )

  parser.add_argument(
    "--r_sub",
    type=int,
    default=1,
    help="",
  )

  parser.add_argument(
    "--rename",
    type=str,
    default="asl_loop",
    help="",
  )

  args = parser.parse_args()
  if args.rename != "no":
    args.map_serialized_path = args.map_serialized_path.replace(
      "full_run_2", args.rename
    )
    args.mesh_path = args.mesh_path.replace("full_run_2", args.rename)
    args.scannet_scene_dir = args.scannet_scene_dir.replace("full_run_2", args.rename)

  label_generator = LabelGenerator(args)
  from scipy.spatial.transform import Rotation as R
  from pathlib import Path

  # H_cam[:3, :3] = H_cam[:3, :3] @ R.from_euler("x", 90, degrees=True).as_matrix()
  # probs = label_generator.get_label(H_cam)
  # import time

  # H_cam[:3, :3] = H_cam[:3, :3] @ R.from_euler("x", -90, degrees=True).as_matrix()
  # probs = label_generator.get_label(H_cam)
  # time.sleep(20)
  # H_cam[:3, :3] = H_cam[:3, :3] @ R.from_euler("y", 90, degrees=True).as_matrix()
  # probs = label_generator.get_label(H_cam)
  # time.sleep(20)
  x = 0
  y = 0
  z = 0
  txts = [str(s) for s in Path(f"{args.scannet_scene_dir}/pose/").rglob("*.txt")]
  for txt in txts:
    H_cam = np.loadtxt(txt)
    out = txt.split("/")[-1][:-4]
    probs = label_generator.get_label(H_cam)
    outt = args.scannet_scene_dir + "/pseudo_label/" + out + ".png"

    # img = imageio.imwrite(outt, np.uint8(np.argmax(probs, axis=2)))
    label_to_png2(probs[:, :, 1:], outt)

  # for i in range(20):

  #   t = [
  #     0,
  #     np.random.randint(-2, 2) * 90,
  #     np.random.randint(-2, 2) * 90,
  #   ]
  #   # print(t)
  #   # H_cam[:3, :3] = H_cam[:3, :3] @ R.from_euler("zxy", t, degrees=True).as_matrix()

  #   from PIL import Image
  #   import cv2

  #   img.show()
