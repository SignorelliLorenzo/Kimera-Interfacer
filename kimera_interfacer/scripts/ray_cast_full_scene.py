from ray_cast_mesh_pyembree import LabelGenerator
import argparse
from pathlib import Path
from PIL import Image
import numpy as np
import os

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--root", type=str, default="/home/jonfrey/Datasets/scannet", help="")
  parser.add_argument("--scene_path", type=str, default="/home/jonfrey/Datasets/scannet_scene_0000", help="")
  parser.add_argument("--mode", type=str, default="map_probs",
                      choices=["gt", "network_prediction", "map_onehot", "map_probs"], help="")
  parser.add_argument("--map_serialized", type=str,
                      default="/home/jonfrey/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/mesh_results/serialized.data",
                      help="")
  parser.add_argument("--mapping_scannet", type=str,
                      default="/home/jonfrey/Datasets/scannet/scannetv2-labels.combined.tsv",
                      help="")

  parser.add_argument("--mesh_name", type=str, default="predict_mesh", help="")
  arg = parser.parse_args()
  label_generator = LabelGenerator(arg)
  i = 10
  poses = [ str(p) for p in Path(f"{arg.scene_path}/pose/").rglob("*.txt") ]
  poses.sort( key=lambda p: int(p.split('/')[-1][:-4]))
  for p in poses:

    index = p.split('/')[-1][:-4]

    if int( index) % 10 != 0:
      continue

    print(index)
    H_cam = np.loadtxt(p)
    map_onehot_label, img, probs = label_generator.get_label(H_cam, i, visu=False, override_mode="map_onehot")
    p_out = os.path.join(arg.scene_path, "iteration/0/map_onehot_label", f"{index}.png" )
    Image.fromarray( np.uint8( map_onehot_label) ).save( p_out)

    p_out = os.path.join(arg.scene_path, "iteration/0/map_onehot_label", f"{index}_img.jpg" )
    Image.fromarray( np.uint8( img) ).save( p_out)

    map_probabilistic_label, img, probs = label_generator.get_label(H_cam, i, visu=False, override_mode="map_probs")
    p_out = os.path.join(arg.scene_path, "iteration/0/map_probabilistic_label", f"{index}.png" )
    Image.fromarray( np.uint8( map_probabilistic_label ) ).save( p_out)

    p_out = os.path.join(arg.scene_path, "iteration/0/map_probabilistic_label", f"{index}_img.jpg" )
    Image.fromarray( np.uint8( img) ).save( p_out)

    p_out = os.path.join(arg.scene_path, "iteration/0/map_probabilistic_soft", f"{index}.npy" )
    np.save( p_out, probs)

    gt_label, img, probs = label_generator.get_label(H_cam, index, visu=False, override_mode="gt")
    detectron_label, img, probs = label_generator.get_label(H_cam, index, visu=False, override_mode="network_prediction")
    def get_acc( pred, target):
      m = np.logical_and(pred != 0, target != 0)
      return (pred[m]==target[m]).sum() / m.sum()

    print(" Detectron: ", get_acc( detectron_label, gt_label),
          " Porbabilistic: ", get_acc( map_probabilistic_label, gt_label),
          " Onehot: ", get_acc( map_onehot_label, gt_label) )
