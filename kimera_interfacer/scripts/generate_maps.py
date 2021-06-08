import yaml
import os
import time
with open("/home/jonfrey/ASL/cfg/exp/create_newlabels/create_load_model.yml") as file:
	exp = yaml.load(file, Loader=yaml.FullLoader)
  
label_identifier = exp['label_generation']['identifier']
scenes = exp['label_generation']['scenes']
print(scenes)

for s in scenes:
  cmd = f"roslaunch kimera_interfacer predict_generic_scene.launch label_identifier:={label_identifier} scene:={s}"
  print(cmd)
  os.system(cmd)
  time.sleep(2)
  