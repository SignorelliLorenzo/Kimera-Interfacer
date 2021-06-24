import numpy as np
import os
from pathlib import Path
from multiprocessing import Pool

from label_generation import LabelGenerator
import imageio

def label_to_png2(label,path):
  max_classes = 40
  H,W,_ = label.shape 
  idxs = np.zeros( (3, H,W) ,dtype=np.uint8)
  values = np.zeros( (3, H,W) )
  label_c = label #.clone()
  max_val_10bit = 1023

  for i in range(3):
    idx = np.argmax( label_c, axis=2 )
    idxs[i] = idx.astype(np.int32)
    
    m = np.eye(max_classes)[idx] == 1
    values[i] = ( (label_c[m] *  max_val_10bit).reshape(H,W)).astype(np.int32)
    values[i][values[i] > max_val_10bit] = max_val_10bit
    label_c[m] = -0.1
  
  values = values.astype( np.int32)
  idxs = idxs.astype( np.int32)
  
  png = np.zeros( (H,W,4), dtype=np.int32 )
  for i in range(3):
    png[:,:,i] = values[i]
    png[:,:,i] = np.bitwise_or( png[:,:,i], idxs[i] << 10 )
  imageio.imwrite(path, png.astype(np.uint16),  format='PNG-FI', compression=9) 
  
  
  print("Done")
  return True

if __name__ == "__main__":
  import argparse  
  parser = argparse.ArgumentParser()
  # EXTERNAL DATA PATHS
  parser.add_argument("--scannet_scene_dir", type=str,
                      default="/home/jonfrey/Datasets/scannet/scans/scene0003_00", help="")
  parser.add_argument("--mesh_path", type=str,
                      default="/home/jonfrey/Datasets/output_kimera_semantics/scene0003_00_labels_pcmr_confidence_05_fixed_epochs_predict_mesh.ply", help="")
  parser.add_argument("--map_serialized_path", type=str,
                      default="/home/jonfrey/Datasets/output_kimera_semantics/scene0003_00_labels_pcmr_confidence_05_fixed_epochs_serialized.data", help="")
  
  parser.add_argument("--label_generate_idtf", type=str,
                    default="debug_reprojected", help="")
  parser.add_argument("--label_generate_dir", type=str,
                    default="/home/jonfrey/Datasets/labels_generated", help="")
    
  args = parser.parse_args()
  
  label_generator = LabelGenerator(args,  visu3d = False)
  
  out_dir = os.path.join( args.label_generate_dir,
                        args.label_generate_idtf, 'scans',
                        args.scannet_scene_dir.split('/')[-1],
                        args.label_generate_idtf )
  Path(out_dir).mkdir(parents=True, exist_ok=True)
  
  
  poses = [ str(p) for p in Path(f"{args.scannet_scene_dir}/pose/").rglob("*.txt") if int(str(p).split('/')[-1][:-4]) % 10 == 0 ]
  poses.sort( key=lambda p: int(p.split('/')[-1][:-4]))
  nr = len(poses)
  
  max_cores = 10
  # with Pool(processes = max_cores) as pool:
  alive_p = []
  for j, p in enumerate( poses) :
    H_cam = np.loadtxt(p)
    probs = label_generator.get_label( H_cam )
    p_out = os.path.join( out_dir, p.split('/')[-1][:-4] + '.png' )
    label_to_png2( probs[:,:,1:] , p_out )
    
    # _res = pool.apply_async(func= label_to_png2, args=(probs[:,:,1:] , p_out) )
    # if j % max_cores == 0 and j != 0:
    #   _res.get()
    print(f"{j}/{nr}")
    
    # _res.get()