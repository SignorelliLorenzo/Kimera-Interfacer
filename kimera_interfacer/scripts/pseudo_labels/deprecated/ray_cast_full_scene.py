
import argparse
from pathlib import Path
import numpy as np
import os
import torch

from ray_cast_mesh_pyembree import LabelGenerator
from convert_label import label_to_png

def gen_and_store( mode, store, index, H_cam, out_dir):
  label, img, probs = label_generator.get_label(H_cam, int(index), visu=False, override_mode=mode)
  if store and mode != "gt":
    p_out = os.path.join(out_dir, f"{index}.png" )
    label_to_png(torch.from_numpy( probs[:,:,1:] ), p_out)
  return label

def get_acc(pred, target):
  m = np.logical_and(pred != 0, target != 0)
  return (pred[m]==target[m]).sum() / m.sum()

def print_avg(acc_out, keys):
  res = np.array( acc_out )
  average = np.mean( res, axis=0 )
  ss = "AVERAGE: | "
  for j,k in enumerate(keys):
    ss = ss + k + ": " + str(int( average[j]*100 ) ) + " | "
  print( ss )
  return res

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  """
  python3 ray_cast_full_scene.py \
  --label_scene_dir=/home/jonfrey/Datasets/labels_generated/prob09_run/scene0000_00/prob09_run \
  --label_generated_idtf=prob09_run_reprojected
  """
  
  # CONFIGURATION
  parser.add_argument("--change_scene", type=str, default="scene0000_00", help="Change scene in all paths!")

  parser.add_argument("--mode", type=str, default="map_probs",
                      choices=["gt", "network_prediction", "map_onehot", "map_probs"], help="")
  parser.add_argument("--confidence", type=float, default=0.5,
                      help="Minimum confidence necessary only use in map_probs_with_confidence")
  # EXTERNAL DATA PATHS
  parser.add_argument("--scannet_scene_dir", type=str,
                      default="/home/jonfrey/Datasets/scannet/scans/scene0000_00", help="")
  parser.add_argument("--label_scene_dir", type=str,
                      default="/home/jonfrey/Datasets/labels_generated/prob09_run/scene0000_00/prob09_run", help="")
  parser.add_argument("--mapping_scannet_path", type=str,
                      default="/home/jonfrey/Datasets/scannet/scannetv2-labels.combined.tsv", help="")
  parser.add_argument("--mesh_path", type=str,
                      default="/home/jonfrey/Datasets/output_kimera_semantics/scene0000_00_predict_mesh.ply", help="")
  parser.add_argument("--map_serialized_path", type=str,
                      default="/home/jonfrey/Datasets/output_kimera_semantics/scene0000_00_serialized.data", help="")

  # PRIVATE
  parser.add_argument("--store", type=bool,
                      default=True,
                      help="")
  parser.add_argument("--label_generated_dir", type=str,
                      default="/home/jonfrey/Datasets/labels_generated", help="")
  parser.add_argument("--label_generated_idtf", type=str,
                      default="prob09_run_reprojected", help="")
  arg = parser.parse_args()

  if arg.change_scene != "scene0000_00":
    arg.scannet_scene_dir = arg.scannet_scene_dir.replace("scene0000_00", arg.change_scene)
    arg.label_scene_dir = arg.label_scene_dir.replace("scene0000_00", arg.change_scene)
    arg.mesh_path = arg.mesh_path.replace("scene0000_00", arg.change_scene)
    arg.map_serialized_path = arg.map_serialized_path.replace("scene0000_00", arg.change_scene)

  out_dir = os.path.join( arg.label_generated_dir,
                         arg.label_generated_idtf, 'scans',
                         arg.scannet_scene_dir.split('/')[-1],
                         arg.label_generated_idtf )
  Path(out_dir).mkdir(parents=True, exist_ok=True)

  gen = ["map_onehot",  "map_probs", "map_probs_with_confidence", "gt", "network_prediction"]
  # gen = ["map_probs", "gt", "network_prediction", "map_probs_with_confidence"]
  gen = [f"map_probs_with_confidence_{arg.confidence}", "gt"]

  for k in gen:
    Path(os.path.join( out_dir,k)).mkdir(parents=True, exist_ok=True)

  label_generator = LabelGenerator(arg)

  poses = [ str(p) for p in Path(f"{arg.scannet_scene_dir}/pose/").rglob("*.txt") if int(str(p).split('/')[-1][:-4]) % 10 == 0 ]
  poses.sort( key=lambda p: int(p.split('/')[-1][:-4]))

  max_el = len(poses)

  # PERPARE OUTPUT BUFFER LIST
  acc_out = []
  import time; st = time.time()
  for j,p in enumerate( poses):
    index = int( p.split('/')[-1][:-4])
    H_cam = np.loadtxt(p)
    try:
      ret_labels= { m: gen_and_store( m, arg.store, index, H_cam, out_dir) for m in gen }
    except Exception as e:
      print("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR" )
      print("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR" )
      print("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR" )
      print("Exception:", e)
      print("index", index)
      print("H_Cam", H_cam)
      print("Arg-store", arg.store)
      print("p", p)
      print("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR" )
      print("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR" )
      print("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR" )

  #   # LOGGING AND ACCURRACY
  #   keys = list( ret_labels.keys() )
  #   keys.sort()
  #   ss = ""
  #   ls = []
  #   for k in keys:
  #     ls.append(get_acc( ret_labels[k], ret_labels["gt"] ) )
  #     ss += k
  #     ss += ": " + str(int( ls[-1]*100)) + "  "
  #   # STORING RESULTS
  #   acc_out.append( ls )
  #   # LOGGING PROGRESS TO CONSOLE
  #   if j% 10 == 0 and j != 0:
  #     print( f"{j}/{max_el}:  {ss}" )
  #     print("Delta t:", time.time()-st, "s")
  #     print("Time left t:", (time.time()-st)/j*(max_el-j), "s"  )  
  #     print_avg(acc_out, keys)

  # res = print_avg(acc_out, keys)
  # if arg.store:
  #   p_out = os.path.join(out_dir, "resulting_acc_raycasting.npy")
  #   np.save(p_out, res)