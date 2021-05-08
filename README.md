# Idee:

write C++ Wrapper that allows the following:

1. Listen to topic with Image, Depth and Semantic Segmentation, CameraTopic
2. 
3. Check if frames align with header_stamp
4. Creating TsdfServer

kimera::PointCloudFromDepth pcl_from_depth;

kimera::PointCloud::Ptr pcl;
pcl = pcl_from_depth.imageCb(depth_img, semantic_img, cam_info);

load_gt_transformation


tsdf_server->processPointCloudMessageAndInsert(pcl, T_G_C, false);