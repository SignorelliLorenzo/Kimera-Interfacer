#!/usr/bin/python3

import os
import sys
path = os.popen("rospack find kimera_semantics_ros").read()
sys.path.append( os.path.join(path, "include/proto") )
import semantic_map_pb2
from google.protobuf.internal.decoder import _DecodeVarint32
import google.protobuf

# def parse( f, msg):
#     buf = f.read(100) # Maximum length of length prefix
#     while buf:
#       msg_len, new_pos = _DecodeVarint32(buf, 0)
#       buf = buf[new_pos:]
#       buf += f.read(msg_len - len(buf))
#       read_row = msg
#       read_row.ParseFromString(buf)
#       buf = buf[msg_len:]
#       buf += f.read(100 - len(buf))
#     return msg

def parse(f,msg):
  buf = f.read()
  n = 0
  while n < len(buf):
    msg_len, new_pos = _DecodeVarint32(buf, n)
    n = new_pos
    msg_buf = buf[n:n+msg_len]
    n += msg_len
    msg = semantic_map_pb2.SemanticMapProto()
    msg.ParseFromString(msg_buf)
  return msg

if __name__ == "__main__":
  file_path = "/home/jonfrey/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/mesh_results/serialized.data"
  msg = semantic_map_pb2.SemanticMapProto()
  with open(file_path, 'rb') as f:
    res = parse(f, msg)

  # print( res.semantic_blocks )
  #   buf = f.read()
  #   msg.ParseFromString(buf)
  # print(msg.semantic_blocks)ro

  #   res = parse(f, msg)
  # print(res)
  print( "origin", res.semantic_blocks[0].origin.y )
  # print( "origin", res.semantic_blocks[0].origin.y )
  # print( "origin", res.semantic_blocks[0].origin.z )
  # print( "VoxelsPerSide", res.semantic_blocks[0].voxels_per_side )
  # print( "Nr of Voxels", len( res.semantic_blocks[0].semantic_voxels) )
  # print( res.semantic_blocks[0].semantic_voxels[0].color.r )
  # print( res.semantic_blocks[0].semantic_voxels[0].semantic_labels[:] )
  #
  # print("NR semantics block: ", len( res.semantic_blocks ) )
  # mean = 0
  # for i in range(0,len( res.semantic_blocks ) ):
  #   mean += len( res.semantic_blocks[i].semantic_voxels )
  # print("Nr average semantic voxels ", mean/len( res.semantic_blocks ) )
  # print("NR semantics blocks: ", len( res.semantic_blocks ) )