import yaml
import os
import time
with open("/home/jonfrey/ASL/cfg/exp/create_newlabels/create_load_model.yml") as file:
  exp = yaml.load(file, Loader=yaml.FullLoader)
  
label_identifier = exp['label_generation']['identifier']
scenes = exp['label_generation']['scenes']

label_generate_idtf = exp['label_generation']['identifier']+"_reprojected"
print(scenes)

output_dir = "/home/jonfrey/Datasets/output_kimera_semantics"
for s in scenes:
  args = {
    "scannet_scene_dir" : f"/home/jonfrey/Datasets/scannet/scans/{s}",
    "mesh_path": f"{output_dir}/{s}_{label_identifier}_predict_mesh.ply",
    "map_serialized_path": f"{output_dir}/{s}_{label_identifier}_serialized.data",
    "label_generate_idtf": label_generate_idtf,
    "label_generate_dir": "/home/jonfrey/Datasets/labels_generated"
  } 
  args_str = ""
  for k,v in args.items():
    args_str += f"--{k}={v} "


  cmd = "/home/jonfrey/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/scripts/pseudo_labels/ray_cast_full_scene_simple.py "
  cmd += args_str
  print(cmd)
  os.system(cmd)
  time.sleep(2)
  