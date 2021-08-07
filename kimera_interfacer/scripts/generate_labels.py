import yaml
import os
import time

import time
import argparse

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument(
    "--exp",
    default="/home/jonfrey/ASL/cfg/exp/MA/scannet_self_supervision/create_labels_from_pretrained.yml",
    help="The main experiment yaml file.",
  )
  args = parser.parse_args()
  exp_cfg_path = args.exp

  with open(exp_cfg_path) as file:
    exp = yaml.load(file, Loader=yaml.FullLoader)

  gm = exp.get("generate_maps", {})
  par = gm.get("parallel", False)
  r_sub = gm.get("sub_reprojected", 1)

  label_identifier = exp["label_generation"]["identifier"]
  scenes = exp["label_generation"]["scenes"]

  if gm.get("label_identifier_out", None) is not None:
    label_generate_idtf = gm.get("label_identifier_out", None)
    label_identifier = gm.get("label_identifier_out", None)
  else:
    label_generate_idtf = exp["label_generation"]["identifier"] + "_reprojected"
  print(scenes)

  output_dir = "/home/jonfrey/Datasets/output_kimera_semantics"
  for s in scenes:
    args = {
      "scannet_scene_dir": f"/home/jonfrey/Datasets/scannet/scans/{s}",
      "mesh_path": f"{output_dir}/{s}_{label_identifier}_predict_mesh.ply",
      "map_serialized_path": f"{output_dir}/{s}_{label_identifier}_serialized.data",
      "label_generate_idtf": label_generate_idtf,
      "label_generate_dir": "/home/jonfrey/Datasets/labels_generated",
      "r_sub": r_sub,
    }
    args_str = ""
    for k, v in args.items():
      args_str += f"--{k}={v} "

    cmd = "/home/jonfrey/miniconda3/envs/track4/bin/python pseudo_labels/ray_cast_full_scene_simple.py "
    cmd += args_str
    if par:
      cmd += " &"
    print(cmd)
    os.system(
      "cd /home/jonfrey/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/scripts && "
      + cmd
    )
    time.sleep(2)
