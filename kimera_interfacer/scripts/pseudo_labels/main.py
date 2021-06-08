if __name__ == "__main__":
  import argparse
  import numpy as np
  
  from label_generation import LabelGenerator
  parser = argparse.ArgumentParser()
  # EXTERNAL DATA PATHS
  parser.add_argument("--scannet_scene_dir", type=str,
                      default="/home/jonfrey/Datasets/scannet/scans/scene0003_00", help="")
  parser.add_argument("--mesh_path", type=str,
                      default="/home/jonfrey/Datasets/output_kimera_semantics/scene0003_01_labels_pcmr_confidence_05_fixed_epochs_predict_mesh.ply", help="")
  parser.add_argument("--map_serialized_path", type=str,
                      default="/home/jonfrey/Datasets/output_kimera_semantics/scene0003_01_labels_pcmr_confidence_05_fixed_epochs_serialized.data", help="")
  
  args = parser.parse_args()
  
  label_generator = LabelGenerator(args)
  i = 10
  H_cam = np.loadtxt(f"{args.scannet_scene_dir}/pose/{i}.txt")
  probs = label_generator.get_label( H_cam )
  print("Done")