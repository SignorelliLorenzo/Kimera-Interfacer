import yaml
import os
import time
with open("/home/jonfrey/ASL/cfg/exp/create_newlabels/create_labels_scene.yml") as file:
  exp = yaml.load(file, Loader=yaml.FullLoader)
  
label_identifier = exp['label_generation']['identifier']
scenes = exp['label_generation']['scenes']
print(scenes, label_identifier)

params = {  "prob_main": 0,
            "label_identifier": label_identifier,
            "fps": 5,
            "prob_aux": 0
          #, "frame_limit": 10
}
par = ""
for k,v in params.items():
  par += f" {k}:={v}  "


label_generate_idtf = exp['label_generation']['identifier']+"_reprojected"
output_dir = "/home/jonfrey/Datasets/output_kimera_semantics"


for j,s in enumerate(scenes):
  aux_labels = "invalid" # f"/home/jonfrey/Datasets/labels_generated/labels_deeplab/scans/{s}/labels_deeplab" #
  cmd = f"roslaunch kimera_interfacer predict_generic_scene.launch scene:={s} aux_labels:={aux_labels}" + par
  print(cmd)
  os.system(cmd)

  
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


  cmd = "/home/jonfrey/miniconda3/envs/track4/bin/python pseudo_labels/ray_cast_full_scene_simple.py "
  cmd += args_str
  print(cmd)
  os.system("cd /home/jonfrey/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/scripts && " + cmd)
  time.sleep(2)
  

