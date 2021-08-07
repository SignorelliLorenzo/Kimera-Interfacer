import yaml
import os
import time
import argparse
import notify2


def sendmessage(title, message):
  notify2.init("Test")
  notice = notify2.Notification(title, message)
  notice.show()
  return


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

  label_identifier = exp["label_generation"]["identifier"]
  scenes = exp["label_generation"]["scenes"]
  print(scenes, label_identifier)

  gm = exp.get("generate_maps", {})

  params = {
    "prob_main": gm.get("certainty_thershold", 0),
    "label_identifier": label_identifier,
    "fps": gm.get("fps", 0.5),
    "prob_aux": 0,
    "voxel_size": gm.get("voxel_size", 0.02),
    "label_identifier_out": gm.get("label_identifier_out", label_identifier),
    "sub_reprojected": gm.get("sub_reprojected", 1),
  }
  print(params)

  par = ""
  for k, v in params.items():
    par += f" {k}:={v}  "

  for j, s in enumerate(scenes):
    aux_labels = "invalid"
    cmd = (
      f"roslaunch kimera_interfacer predict_generic_scene.launch scene:={s} aux_labels:={aux_labels}"
      + par
    )

    print(cmd)
    os.system(cmd)

    leng = len(scenes)
    sendmessage("Finished Generate Map", f"{j}/{leng}: {s}")
    time.sleep(10)
