"""
TODO: Jonas Frey

This needs a complete refactoring.
Integrate LabelLoaderAuto at first.
Split the visualization into a seperat module with o3d 
Have a utils file for everything related to the voxels map.
Implement a seperat map loader
Implement a rendered where the map can be passed
"""

import os
os.system("export LD_LIBRARY_PATH=/usr/local/lib")

import argparse
import copy

import imageio
import numpy as np
import open3d as o3d
import pandas
import rospkg
import torch
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector
import matplotlib.pyplot as plt


from convert_label import png_to_label

# PROTOBUF semantic map
import semantic_map_pb2
from google.protobuf.internal.decoder import _DecodeVarint32


def parse( file_handle, msg):
  buf = file_handle.read()
  n = 0
  while n < len(buf):
    msg_len, new_pos = _DecodeVarint32(buf, n)
    n = new_pos
    msg_buf = buf[n:n+msg_len]
    n += msg_len
    read_metric = msg
    read_metric.ParseFromString(msg_buf)
  return msg

def get_semantic_map(p):
  msg = semantic_map_pb2.SemanticMapProto()
  with open(p, 'rb') as f:
    msg = parse(f, msg)
  return msg


def get_rays(k, size, extrinsic, d_min=0.1, d_max=5):
  h, w = size
  v, u = np.mgrid[0:h, 0:w]
  n = np.ones_like(u)
  image_coordinates = np.stack([u, v, n], axis=2).reshape((-1, 3))
  k_inv = np.linalg.inv(k)
  points = (k_inv @ image_coordinates.T).T

  start = copy.deepcopy(points) * d_min
  stop = copy.deepcopy(points) * d_max
  dir = stop - start

  start = start.reshape((h, w, 3))
  stop = stop.reshape((h, w, 3))
  dir = dir.reshape((h, w, 3))
  return start, stop, dir


def transform(points, H):
  return points @ H[:3, :3].T + H[:3, 3]


def get_mapping_scannet(p):
  df = pandas.read_csv(p, sep='\t')
  mapping_source = np.array(df['id'])
  mapping_target = np.array(df['nyu40id'])
  mapping_scannet = torch.zeros((int(mapping_source.max() + 1)), dtype=torch.int64)
  for so, ta in zip(mapping_source, mapping_target):
    mapping_scannet[so] = ta
  return mapping_scannet


def get_a_b_c_from_linear(index, voxels_per_side):
  a = index % (voxels_per_side)
  b = ((index - a) / (voxels_per_side)) % (voxels_per_side)
  c = ((index - a) - (voxels_per_side * b)) / (voxels_per_side ** 2)
  return (int(a), int(b), int(c))


def get_linear_from_a_b_c(a, b, c, voxels_per_side):
  return a + (b * voxels_per_side) + (c * voxels_per_side * voxels_per_side)


def get_x_y_z(index, origin, voxels_per_side, voxel_size):
  abc = get_a_b_c_from_linear(index, voxels_per_side)
  print("abc", abc)
  add = np.full((3), voxel_size) * (abc - np.array([16, 16, 16]))
  return origin + add


eps = 10 ** (-4)

def get_grid_index_from_point(point, grid_size_inv):
  return np.floor(point * grid_size_inv + eps).astype(np.uint32)


def get_semantic_voxel_from_point(point, voxel_size, voxels_per_side):
  grid_size = voxel_size * voxels_per_side
  grid_size_inv = 1 / grid_size
  block_coordinate = get_grid_index_from_point(point, grid_size_inv)
  point_local = point - block_coordinate * np.fill(grid_size, 3)
  local_coordinate = get_grid_index_from_point(point_local, 1 / voxel_size)
  return block_coordinate, local_coordinate


# What happens if voxel is not initalized
# We dont have a good way in the protobuff format to access the voxels by keys !
def parse_protobug_msg_into_accessiable_np_array(map):
  """
  Assumes constant voxel size and grid.
  Will allocate the full memory in a cube
  :param origins:
  :param map:
  :return:
  """
  voxels_per_side = map.semantic_blocks[0].voxels_per_side
  voxel_size = map.semantic_blocks[0].voxel_size

  origins = np.zeros((len(map.semantic_blocks), 3))
  for i in range(len(map.semantic_blocks)):
    origins[i, 0] = map.semantic_blocks[i].origin.x
    origins[i, 1] = map.semantic_blocks[i].origin.y
    origins[i, 2] = map.semantic_blocks[i].origin.z

  # Real mi and ma value of voxel_grid
  mi = np.min(origins, axis=0)
  ma = np.max(origins, axis=0) + (voxel_size * voxels_per_side)

  large_grid = voxel_size * voxels_per_side
  large_grid_inv = 1 / large_grid
  elements = np.floor(((ma - mi) + eps) / large_grid).astype(np.uint32)

  # Store all voxels in a grid volume
  voxels = np.zeros((*tuple((elements) * voxels_per_side), 41))

  for j, block in enumerate(map.semantic_blocks):
    block_idx = get_grid_index_from_point(origins[j] - mi, large_grid_inv)
    block_idx = block_idx * voxels_per_side
    for sem_voxel in block.semantic_voxels:
      abc = get_a_b_c_from_linear(sem_voxel.linear_index, voxels_per_side)
      voxel_idx = block_idx + abc
      voxels[tuple(voxel_idx)] = sem_voxel.semantic_labels
  # dont return the orgin of the first block which is center
  # return orging point of new voxel_grid
  return voxels, mi

def draw_cube(vis, translation, size, color):
  cube = o3d.geometry.TriangleMesh.create_box()
  cube.scale(size, center=cube.get_center())
  cube.translate(translation, relative=False)
  cube.paint_uniform_color(color / 255)
  vis.add_geometry(cube)

def get_voxels_grid_idx_from_point(point, mi, voxel_size):
  idx = np.floor((point - mi) / (voxel_size))
  return idx.astype(np.uint32)

def load_label_scannet( p, mapping_scannet):
  label_gt = imageio.imread( p )
  label_gt = torch.from_numpy(label_gt.astype(np.int32)).type(
    torch.float32)[:, :]  # H W
  sa = label_gt.shape
  label_gt = label_gt.flatten()
  label_gt = mapping_scannet[label_gt.type(torch.int64)]
  label_gt = label_gt.reshape(sa) # 1 == chairs 40 other prop  0 invalid
  return label_gt.numpy()

def load_label_network( p, ignore):
  label_gt = imageio.imread( p )
  return label_gt

def load_label_network_new_format( p, ignore):
  label_gt = png_to_label(p)
  return np.argmax( label_gt, axis= 2) + 1 
  # return label_gt

# TODO Jonas Frey create data_connector to split rendering from loading
class LabelGenerator:
  def __init__(self, arg):
    self.arg = arg

    # Pass the args config
    scannet_scene_dir = arg.scannet_scene_dir
    label_scene_dir = arg.label_scene_dir
    mapping_scannet_path = arg.mapping_scannet_path
    self._mesh_path = arg.mesh_path
    map_serialized_path = arg.map_serialized_path

    self._mode = arg.mode
    self._confidence = arg.confidence

    self._gt_dir = f"{scannet_scene_dir}/label-filt/"
    self._label_scene_dir = label_scene_dir
    self._mapping_scannet = get_mapping_scannet( mapping_scannet_path)

    rospack = rospkg.RosPack()
    kimera_interfacer_path = rospack.get_path('kimera_interfacer')
    # MAPPING
    mapping = np.genfromtxt(f'{kimera_interfacer_path}/cfg/nyu40_segmentation_mapping.csv', delimiter=',')
    ids = mapping[1:, 5]
    self._rgb = mapping[1:, 1:4]
    self._rgb[0, :] = 255

    mesh = trimesh.load(self._mesh_path)

    # GT LABEL TEST
    label_gt = imageio.imread( os.path.join( scannet_scene_dir , "label-filt/0" + '.png' ) )
    H, W = label_gt.shape

    size = (H, W)
    self._size = size
    data = np.loadtxt(f"{ scannet_scene_dir }/intrinsic/intrinsic_depth.txt")
    k_render = np.array([[data[0, 0], 0, data[0, 2]],
                         [0, data[1, 1], data[1, 2]],
                         [0, 0, 1]])
    data = np.loadtxt(f"{scannet_scene_dir}/intrinsic/intrinsic_color.txt")
    k_image = np.array([[data[0, 0], 0, data[0, 2]],
                        [0, data[1, 1], data[1, 2]],
                        [0, 0, 1]])

    start, stop, dir = get_rays(k_image, size, extrinsic=None, d_min=0.3, d_max=1.4)
    self._rmi = RayMeshIntersector(mesh)
    self._start = start
    self._dir = dir

    # Get for onehote encoding the colors in the map
    colors = self._rmi.mesh.visual.face_colors[:,:3]
    self.faces_to_labels = np.zeros( (colors.shape[0] ))
    unique, inverse = np.unique(colors, return_inverse=True, axis= 0)
    for k, c in enumerate( unique):
      self.faces_to_labels[ inverse == k] = np.argmin( np.linalg.norm(self._rgb-c[:3], axis=1,ord=2)  , axis = 0 )

    # Parse serialized voxel_data to usefull numpy structure
    map = get_semantic_map(map_serialized_path)
    self._voxels, self._mi = parse_protobug_msg_into_accessiable_np_array(map)

    self._output_buffer_probs = np.zeros((H, W, self._voxels.shape[3]))
    self._output_buffer_img = np.zeros((H, W, 3))
    self._output_buffer_label = np.zeros((H, W))

    self._voxel_size = map.semantic_blocks[0].voxel_size

    v, u = np.mgrid[0:H, 0:W]
    self._v = v.reshape(-1)
    self._u = u.reshape(-1)

  def get_label(self, H_cam, frame, visu=True, override_mode=None):
    if override_mode is None:
      mode = self._mode
    else:
      mode = override_mode

    if mode == "gt":
      self._output_buffer_label = load_label_scannet( f"{self._gt_dir}/{frame}.png", self._mapping_scannet)
      for i in range(0, 41):
        self._output_buffer_img[self._output_buffer_label == i, :3] = self._rgb[i]
      self._output_buffer_probs.fill(0)
      tup = ()
    elif mode == "network_prediction":
      self._output_buffer_label = load_label_network(
        os.path.join( self._label_scene_dir, f"{frame}.png"),
        self._mapping_scannet)

      for i in range(0, 41):
        self._output_buffer_img[self._output_buffer_label == i, :3] = self._rgb[i]
      self._output_buffer_probs.fill(0)
    elif mode == "map_onehot" or mode == "map_probs" or mode.find("map_probs_with_confidence") != -1:
      self.set_label_raytracing( H_cam, mode, visu)
    else:
      raise ValueError("Invalid mode")

    return self._output_buffer_label, np.uint8( self._output_buffer_img ), self._output_buffer_probs

  def set_label_raytracing(self, H_cam, mode , visu):
    # Move Camera Rays
    ray_origins = transform(self._start.reshape((-1, 3)), H_cam)
    H_turn = np.eye(4)
    H_turn[:3, :3] = H_cam[:3, :3]
    ray_directions = transform(self._dir.reshape((-1, 3)), H_turn)

    # Perform Raytracing
    locations, index_ray, index_tri = self._rmi.intersects_location(ray_origins=ray_origins,
                                                                    ray_directions=ray_directions,
                                                                    multiple_hits=False)
    if mode == "map_onehot":
      colors = self._rmi.mesh.visual.face_colors[ index_tri ]
      self._rmi.mesh.visual.vertex_colors
      # Reset the buffer to invalid
      self._output_buffer_label[:,:] = 0
      tmp = np.copy( self._output_buffer_img )
      for j in range(locations.shape[0]):
        _v, _u = self._v[index_ray[j]], self._u[index_ray[j]]
        tmp[_v, _u, :] = colors[j,:3]
        self._output_buffer_label[_v,_u] = self.faces_to_labels[index_tri[j]]

      for i in range(0, 41):
        self._output_buffer_img[self._output_buffer_label == i, :3] = self._rgb[i]

      if visu:
        plt.imshow( np.uint8(self._output_buffer_img) )
        plt.imshow( np.uint8(tmp) )
      return self._output_buffer_label, self._output_buffer_img, None

    # Compute closest voxel index
    idx_tmp = np.floor(((locations - self._mi + eps) / self._voxel_size)).astype(np.uint32)
    self._output_buffer_probs.fill(0)
    self._output_buffer_probs[:, :, 0] = 1
    # Store class probabilites in buffer
    for j in range(locations.shape[0]):
      _v, _u = self._v[index_ray[j]], self._u[index_ray[j]]
      self._output_buffer_probs[self._v[index_ray[j]], self._u[index_ray[j]], :] = self._voxels[tuple(idx_tmp[j])]

    self._output_buffer_probs = self._output_buffer_probs - \
                                (np.min( self._output_buffer_probs, axis=2)[...,None]).repeat(self._output_buffer_probs.shape[2],2)
    self._output_buffer_probs = self._output_buffer_probs/ \
                                (np.sum( self._output_buffer_probs, axis=2)[...,None]).repeat(self._output_buffer_probs.shape[2],2)

    if mode.find("map_probs_with_confidence") != -1:
      m = self._output_buffer_probs.max(axis=2) < self._confidence
      self._output_buffer_label[ m ] = 0
      self._output_buffer_probs[ m ] = 0
      self._output_buffer_probs[ m, 0 ] = 1

    self._output_buffer_label = np.argmax(self._output_buffer_probs, axis=2)
    self._output_buffer_img.fill(0)
    for i in range(0, 41):
      self._output_buffer_img[self._output_buffer_label == i, :3] = self._rgb[i]

    if visu:
      plt.imshow( np.uint8(self._output_buffer_img) )
      plt.show()
      self.visu_current_buffer(locations, ray_origins)



  def visu_current_buffer(self, locations=None, ray_origins=None, sub=1000, sub2=8):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=self._size[1],
                      height=self._size[0], visible=True)
    mesh_o3d = o3d.io.read_triangle_mesh(self._mesh_path)
    vis.add_geometry(mesh_o3d)

    if locations is not None:
      for j in range(0, locations.shape[0], sub):
        # Draw detected mesh intersection points
        sphere_o3d = o3d.geometry.TriangleMesh.create_sphere(radius=0.01).translate(locations[j, :])
        vis.add_geometry(sphere_o3d)

    if ray_origins is not None:
      # Draw camera rays start and end
      for j in range(0, ray_origins.shape[0], sub):
        sphere_o3d = o3d.geometry.TriangleMesh.create_sphere(radius=0.01).translate(ray_origins[j, :])
        sphere_o3d.paint_uniform_color([1, 0, 0])
        vis.add_geometry(sphere_o3d)

    if False:
      for block in range(len(map.semantic_blocks)):
        large_size = (map.semantic_blocks[block].voxel_size * map.semantic_blocks[block].voxels_per_side)  # in meters
        cube = o3d.geometry.TriangleMesh.create_box()
        cube.scale(1 * large_size / 5, center=cube.get_center())
        cube.translate(origins[block])
        vis.add_geometry(cube)
        print(large_size, origins[block], map.semantic_blocks[block].voxel_size)

    if False:
      for block in range(4):  # len(map.semantic_blocks)):
        voxel_size = map.semantic_blocks[block].voxel_size  # in meters
        voxels_per_side = map.semantic_blocks[block].voxels_per_side

        for j in range(0, len(map.semantic_blocks[block].semantic_voxels), 1):
          cube = o3d.geometry.TriangleMesh.create_box()
          cube.scale(1 * voxel_size * 2, center=cube.get_center())
          index = map.semantic_blocks[block].semantic_voxels[j].linear_index
          trans = get_x_y_z(index, origins[block], voxels_per_side, voxel_size)
          cube.translate(trans)
          rgb = [
            map.semantic_blocks[block].semantic_voxels[j].color.r / 255,
            map.semantic_blocks[block].semantic_voxels[j].color.g / 255,
            map.semantic_blocks[block].semantic_voxels[j].color.b / 255]

          rgb = [0, 0, 1]
          cube.paint_uniform_color(rgb)
          vis.add_geometry(cube)

    if True:
      for j in range(0, self._voxels.shape[0], sub2):
        for k in range(0, self._voxels.shape[1], sub2):
          for l in range(0, self._voxels.shape[2], sub2):
            translation = np.array([j, k, l], dtype=np.float)
            translation *= self._voxel_size
            translation += self._mi

            # check if voxel is valid
            if np.sum(self._voxels[j, k, l, :]) != 0:
              col_index = np.argmax(self._voxels[j, k, l, :])
              draw_cube(vis, translation, self._voxel_size, self._rgb[col_index])

    if locations is not None:
      idx_tmp = np.floor(((locations - self._mi + eps) / self._voxel_size)).astype(np.uint32)

      for j in range(0, locations.shape[0], sub):
        col_index = np.argmax(self._voxels[idx_tmp[j, 0],
                              idx_tmp[j, 1],
                              idx_tmp[j, 2], :])

        translation = np.copy(idx_tmp[j]).astype(np.float)
        translation *= self._voxel_size
        translation += self._mi
        draw_cube(vis, translation, self._voxel_size * 2, self._rgb[col_index])

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
  parser = argparse.ArgumentParser()

  # CONFIGURATION
  parser.add_argument("--mode", type=str, default="map_probs",
                      choices=["gt", "network_prediction", "map_onehot", "map_probs", "map_probs_with_confidence"], help="")
  parser.add_argument("--confidence", type=float, default=0.5,
                      help="Minimum confidence necessary only use in map_probs_with_confidence")

  # EXTERNAL DATA PATHS
  parser.add_argument("--scannet_scene_dir", type=str,
                      default="/home/jonfrey/Datasets/scannet/scans/scene0000_00", help="")
  parser.add_argument("--label_scene_dir", type=str,
                      default="/home/jonfrey/Datasets/labels_generated/pretrain_scene_10-60/scene0000_00/pretrain_scene_10-60", help="")
  parser.add_argument("--mapping_scannet_path", type=str,
                      default="/home/jonfrey/Datasets/scannet/scannetv2-labels.combined.tsv", help="")
  parser.add_argument("--mesh_path", type=str,
                      default="/home/jonfrey/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/mesh_results/predict_mesh.ply", help="")
  parser.add_argument("--map_serialized_path", type=str,
                      default="/home/jonfrey/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/mesh_results/serialized.data", help="")

  parser.add_argument("--mesh_name", type=str, default="predict_mesh", help="")
  
  new_format = True
  if new_format:
    load_label_network = load_label_network_new_format
  
  args = parser.parse_args()
  label_generator = LabelGenerator(args)
  i = 10
  H_cam = np.loadtxt(f"{args.scannet_scene_dir}/pose/{i}.txt")
  label,img, probs = label_generator.get_label(H_cam, i)