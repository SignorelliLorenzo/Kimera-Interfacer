import trimesh
import numpy as np

p = "/home/jonas/catkin_ws/src/Kimera-Interfacer/kimera_interfacer/mesh_results/mesh.ply"

import copy
import numpy as np
import open3d as o3d
import cv2 as cv
import matplotlib.pyplot  as plt

if __name__ == "__main__":
    mesh = o3d.io.read_triangle_mesh(p)
    cam = o3d.camera.PinholeCameraIntrinsic()
    gt_dir ="/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/label-filt/"
    data = np.loadtxt("/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/intrinsic/intrinsic_depth.txt")

    print("Try to render a mesh with normals (exist: " +
          str(mesh.has_vertex_normals()) + ") and colors (exist: " +
          str(mesh.has_vertex_colors()) + ")")

    vis = o3d.visualization.Visualizer()
    vis.create_window(width=640,
                      height=480,visible=False)
    vis.add_geometry( mesh )
    K_render = np.array( [[data[0,0],0, data[0,2] ],
                          [0,data[1,1], data[1,2] ],
                          [0,0,1] ] )

    # map1, map2 = cv.initUndistortRectifyMap(
    #     K_render,
    #     np.array([0,0,0,0]),
    #     np.eye(3),
    #     data[:3,:3], (640,480), cv.CV_32FC1)
    #recitfy between cameras
    # img_post = cv.remap( img_pre,
    #                      map1,
    #                      map2,
    #                      interpolation=cv.INTER_NEAREST,
    #                      borderMode=cv.BORDER_CONSTANT,
    #                      borderValue=0)
    # plt.imsave(f"post_test_{i}.png", img_post, dpi = 1)

    for i in range(100):
        ctr = vis.get_view_control()
        init_param = ctr.convert_to_pinhole_camera_parameters()
        init_param.intrinsic.width = 640
        init_param.intrinsic.height = 480
        init_param.intrinsic.set_intrinsics( 640,480, K_render[0,0], K_render[1,1], K_render[0,2], K_render[1,2] )
        H_cam = np.loadtxt(f"/home/jonas/Documents/Repos/tsdf-fusion-python/data/scene0000_00/pose/{i}.txt")
        init_param.extrinsic = np.linalg.inv(H_cam)
        ctr.convert_from_pinhole_camera_parameters(init_param, allow_arbitrary=True)
        image = vis.capture_screen_float_buffer(True)
        #plt.imsave(f"init_{i}.png", np.asarray(image), dpi = 1)
        img_pre = np.asarray(image)
        label_gt = imageio.imread( gt_dir + i + '.png' )