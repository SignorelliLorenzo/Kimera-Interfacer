import trimesh
import copy
import numpy as np
import open3d as o3d
import imageio
import cv2 as cv
import rospkg
import matplotlib.pyplot as plt

import os
import pandas
import torch
from PIL import Image

if __name__ == "__main__":
    # Target ACC array
    elements = 5570
    acc_arr = np.zeros( (int(elements/10),2) )
    root = "/home/jonfrey/Datasets/scannet"
    visu = False
    offscreen = True
    rospack = rospkg.RosPack()
    kimera_interfacer_path = rospack.get_path('kimera_interfacer')
    base_path = "/home/jonfrey/Datasets/scannet_scene_0000" # rospy.get_param("~/dl_mock/base_path")
    mesh_name = 'predict_mesh'
    gt_dir = f"{base_path}/label-filt/"
    detectron_dir = f"{base_path}/label_detectron2/"

    # MAPPING
    mapping = np.genfromtxt(f'{kimera_interfacer_path}/cfg/nyu40_segmentation_mapping.csv' , delimiter=',')
    ids = mapping[1:, 5]
    rgb = mapping[1:, 1:4]
    rgb[0,:] = 255

    # MESH
    p = f"{kimera_interfacer_path}/mesh_results/{mesh_name}.ply"
    mesh = o3d.io.read_triangle_mesh(p)
    cam = o3d.camera.PinholeCameraIntrinsic()
    print("Try to render a mesh with normals (exist: " +
          str(mesh.has_vertex_normals()) + ") and colors (exist: " +
          str(mesh.has_vertex_colors()) + ")")

    #MAPPING SCANNET
    tsv = os.path.join(root, "scannetv2-labels.combined.tsv")
    df = pandas.read_csv(tsv, sep='\t')
    mapping_source = np.array( df['id'] )
    mapping_target = np.array( df['nyu40id'] )
    mapping_scannet = torch.zeros( ( int(mapping_source.max()+1) ),dtype=torch.int64)
    for so,ta in zip(mapping_source, mapping_target):
        mapping_scannet[so] = ta

    # CAMERA
    data = np.loadtxt(f"{base_path}/intrinsic/intrinsic_depth.txt")
    K_depth = np.array( [[data[0,0],0, data[0,2] ],
                          [0,data[1,1], data[1,2] ],
                          [0,0,1] ] )
    data = np.loadtxt(f"{base_path}/intrinsic/intrinsic_color.txt")
    K_image = np.array( [[data[0,0],0, data[0,2] ],
                         [0,data[1,1], data[1,2] ],
                         [0,0,1] ] )
    # GT LABEL TEST
    label_gt = imageio.imread( gt_dir + "0" + '.png' )
    H,W = label_gt.shape

    # CREATE RENDERER
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=W,
                      height=H,visible=not offscreen)
    vis.add_geometry( mesh )
    rgb_fancy = rgb[:,None,None,:].repeat( H, 1).repeat(W,2)
    ctr = vis.get_view_control()
    init_param = ctr.convert_to_pinhole_camera_parameters()
    init_param.intrinsic.width = W
    init_param.intrinsic.height = H

    if visu:
        fig1, ax1 = plt.subplots()
        imgplot = ax1.imshow(np.zeros((H,W,3))) # "plot" the image.
        fig1.show()

    if not offscreen:
        frame_id = 0
        class Callback:
            def __init__(self):
                self.frame_id = 0
            def move_forward(self, vis):
                ctr = vis.get_view_control()
                init_param = ctr.convert_to_pinhole_camera_parameters()
                init_param.intrinsic.width = W
                init_param.intrinsic.height = H
                init_param.intrinsic.set_intrinsics( W,H, K_image[0,0], K_image[1,1], K_image[0,2], K_image[1,2] )
                H_cam = np.loadtxt(f"{base_path}/pose/{self.frame_id}.txt")
                init_param.extrinsic = np.linalg.inv(H_cam)
                ctr.convert_from_pinhole_camera_parameters(init_param, allow_arbitrary=True)
                self.frame_id += 1
                return False
        cb = Callback()
        vis.register_animation_callback(cb.move_forward)
        vis.run()
        vis.destroy_window()
    #
    #
    def load_label_detectron( p):
        label_gt = imageio.imread( p )
        return label_gt

    def load_label_scannet( p ):
        label_gt = imageio.imread( p )
        label_gt = torch.from_numpy(label_gt.astype(np.int32)).type(
            torch.float32)[:, :]  # H W
        sa = label_gt.shape
        label_gt = label_gt.flatten()
        label_gt = mapping_scannet[label_gt.type(torch.int64)]
        label_gt = label_gt.reshape(sa) # 1 == chairs 40 other prop  0 invalid
        # label_gt = label_gt - 1
        return label_gt.numpy()

    for i in range(0,elements, 10):
            print(f'Frame {i}/{elements}')

            init_param.intrinsic.set_intrinsics( W,H, K_image[0,0], K_image[1,1], K_image[0,2], K_image[1,2] )
            H_cam = np.loadtxt(f"{base_path}/pose/{i}.txt")
            init_param.extrinsic = np.linalg.inv(H_cam)
            ctr.convert_from_pinhole_camera_parameters(init_param, allow_arbitrary=True)


            if offscreen:
                image = vis.capture_screen_float_buffer(True)
                label_pred_img_dist = np.uint8( np.asarray(image) * 255)[None].repeat(41,0)
                dis = np.abs( label_pred_img_dist - rgb_fancy).sum(axis=3)
                label_pred = np.argmin( dis, axis = 0)

                label_gt = load_label_scannet( gt_dir + str(i) + '.png' )

                label_detectron = load_label_detectron( detectron_dir + str(i) + '.png')

                m = (label_gt != 0)# * (label_pred != 0)
                if np.sum( m ) != 0:
                    acc = np.sum( label_gt[m] == label_pred[m] ) / np.sum( m )
                    acc_arr[int(i/10),0] = acc

                m = (label_gt != 0)# * (label_detectron != 0)
                if np.sum( m ) != 0:
                    acc = np.sum( label_gt[m] == label_detectron[m] ) / np.sum( m )
                    acc_arr[int(i/10),1] = acc

            # res = np.concatenate( [label_pred*5, label_gt*5], axis=1)[:,:,None].repeat(3,2)
                if visu:
                    label_pred_img = np.round( np.asarray(image) * 255)
                    label_gt_img = np.ones( (label_gt.shape[0], label_gt.shape[1], 3) ) * 255
                    for j in range(0, 40):
                        label_gt_img[label_gt == j, :3] = rgb[j]
                    label_gt_img = np.uint8( label_gt_img )
                    label_detectron_img = np.ones( (label_detectron.shape[0], label_detectron.shape[1], 3) ) * 255
                    for j in range(0, 40):
                        label_detectron_img[label_detectron == j, :3] = rgb[j]
                    label_detectron_img = np.uint8( label_detectron_img )
                    res = np.concatenate( [label_pred_img, label_gt_img, label_detectron_img], axis=1)

                    img_store = Image.fromarray(np.uint8(res))
                    img_store.save(f"/home/jonfrey/Datasets/scannet_scene_0000/map_reprojection/{i:06d}.png")
                    imgplot.set_data(np.uint8(res) )
                    fig1.canvas.flush_events()
                    fig1.canvas.draw_idle()

    print("ACC TOTAL: MAP, DETECTRON", acc_arr.mean(axis=0))