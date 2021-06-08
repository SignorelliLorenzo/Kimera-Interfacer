from pathlib import Path
from cv2 import VideoWriter, VideoWriter_fourcc
import numpy as np
pa = Path("/home/jonfrey/tmp")


dic = {}
base_names =  [ "mask_rcnn", "final", "depth_filled", "network", "seg_image" ,"seg_depth"]
for base_name in base_names:


  paths = [str(s) for s in pa.rglob(f"*{base_name}.png")]
  la = lambda p: int(p.split('/')[-1][:p.split('/')[-1].find("__")])
  paths.sort( key = la)

  print(paths)

  dic[base_name] = paths


import cv2
import time
FPS = 5
height,width,_ = np.array( cv2.imread(paths[0] )).shape
fourcc = VideoWriter_fourcc("M", "J", "P","G")
video = VideoWriter(f'/home/jonfrey/Documents/master_thesis/weekly/21_05_31/videos/{base_name}_merged.avi', fourcc, float(FPS), (width*2, height))
print(width,height)
from PIL import Image
for j,p in enumerate( paths ) :
  
  if j > 150:
    break
  
  img = Image.open(dic['network'][j] )
  img2 = Image.open(dic['final'][j] )
  def get_concat_h(im1, im2):
    dst = Image.new('RGB', (im1.width + im2.width, im1.height))
    dst.paste(im1, (0, 0))
    dst.paste(im2, (im1.width, 0))
    return dst
  img = get_concat_h( img, img2)
  r, g, b = img.split()
  img = np.array( Image.merge("RGB", (b, g, r)) )
  print(j)
  video.write(img)
video.release()


