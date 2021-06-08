import yaml
import os

exp_path = "/home/jonfrey/ASL/cfg/exp/create_newlabels/create_load_model.yml"
label_generated_idtf = "labels_pcmr_confidence_05_fixed_epochs_reprojected"


with open(exp_path) as file:
	exp = yaml.load(file, Loader=yaml.FullLoader)

mesh_label_idtf =  exp['label_generation']['identifier']
label_identifier = exp['label_generation']['identifier']
scenes = exp['label_generation']['scenes']

b = "/home/jonfrey/Datasets/labels_generated/"
output_dir = "/home/jonfrey/Datasets/output_kimera_semantics"

for j, s in enumerate(scenes):
  # if s.find("scene0003_01") == -1: continue
  
  args = {
    "mode": "map_probs",
    "confidence": 0,
    "change_scene": s,
    "label_scene_dir": f"{b}{label_identifier}/scans/{s}/{label_identifier}",
    "mesh_path": f"{output_dir}/{s}_{mesh_label_idtf}_predict_mesh.ply",
    "map_serialized_path": f"{output_dir}/{s}_{mesh_label_idtf}_serialized.data",
    "label_generated_idtf": label_generated_idtf
  } 
  args_str = ""
  for k,v in args.items():
    args_str += f"--{k}={v} "

  gen_labels = f"python3 ray_cast_full_scene.py {args_str}"
  print( gen_labels )
  os.system(gen_labels)
  
  
"""
python3 ray_cast_full_scene.py --mode=map_probs --confidence=0 --change_scene=scene0003_00 --label_scene_dir=/home/jonfrey/Datasets/labels_generated/labels_pretrain25k_correct_mapping/scans/scene0001_00/labels_pretrain25k_correct_mapping --mesh_path=/home/jonfrey/Datasets/output_kimera_semantics/scene0001_00_labels_pretrain25k_correct_mapping_predict_mesh.ply --map_serialized_path=/home/jonfrey/Datasets/output_kimera_semantics/scene0001_00_labels_pretrain25k_correct_mapping_serialized.data --label_generated_idtf=labels_pretrain25k_correct_mapping_reprojected 
"""