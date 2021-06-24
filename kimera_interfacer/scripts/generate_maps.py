import yaml
import os
import time
with open("/home/jonfrey/ASL/cfg/exp/create_newlabels/create_load_model.yml") as file:
	exp = yaml.load(file, Loader=yaml.FullLoader)
  
label_identifier = exp['label_generation']['identifier']
scenes = exp['label_generation']['scenes']
print(scenes, label_identifier)



params = {  "prob_main": 0.3,
            "label_identifier": label_identifier,
            "fps": 5,
            "prob_aux": 0.3
          #, "frame_limit": 10
}
par = ""
for k,v in params.items():
  par += f" {k}:={v}  "

for j,s in enumerate(scenes):
  aux_labels = f"/home/jonfrey/Datasets/labels_generated/labels_deeplab/scans/{s}/labels_deeplab" #"invalid"
  cmd = f"roslaunch kimera_interfacer predict_generic_scene.launch scene:={s} aux_labels:={aux_labels}" + par
  print(cmd)
  os.system(cmd)
  time.sleep(20)