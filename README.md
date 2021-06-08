## Idee:

Write C++ wrapper for Kimera-Semantics:

1. Publish network outputs vis PythonRosNode with aligned header_stamps
2. Integrate network output with NYU40 labels using Kimera-Semantics server
3. Keep track of the full posterior over class probabilities for each voxel
4. Extract a mesh using marching cubes algo. from the TSDF
5. Store the TSDF Volume with the semantic labels (using protobuf for fast serialization)
6. Load the mesh and full_tsdf_volume
7. Perform ray-tracing using Intel Embree kernals for each original camera position
8. Use calculated camera-ray to mesh intersection to get probability from closes voxel
9. Store the back-projected label from the map as a soft and hard label

-> Go back and retrain network with advanced labels

## Repository:
`kimera_interfacer/scripts/dl_mock.py` Publishes network output:

`kimera_interfacer/scripts/dl_mock.py` Publishes network output:

`kimera_interfacer/launch/gt.launch` Launch with gt map.
`kimera_interfacer/launch/predict.launch` Launch with network prediction. (Detectorn 2)

`kimera_interfacer/scripts/ray_cast_mesh_pyembree.py` RayCasting. Needs to be rewritten as a node offering a service.

## Further: 
To run this you need to install my Kimera Semantic Fork.
A features in the original version do not exist:
NYU40 labels support as a configuration parameter.
Serialization fo the TSDF with label class probabilities




## Current results:

Reprojecting back the labels to the original camera frames leads to less accurate labels!

What we in best case want:  
Method:
  - **Input:**  Camera poses, depth, images, labels (by current CL-Model(FastSCNN))
  - **Output:** Label for the image with increased acc.
  - **Optional:** Other supervision signal by different networks.

What problems we can solve with this:
  - Spatial and temporal consistency of the observations!

Limitations:
- Discovering new classes, domain adaptation (one dataset labels bicycles as "vehicles" in the other as "properties")


What to do next:
1. Use multiple networks to probabilistically accumulated teacher supervision signal (YOLO, Detectron, MaskCNN, ScanNet).
2. Make the reprojection the the labels better. Use depth and color information to apply CRF or use superpixels.
3. Refine the voxalized map.   
   Simply taking the argmax of each voxel probability is not the best we can do.  
   Use learning network to segment the voxalized map! Getting to the notion of voxels belonging to a certain object.  
4. Keeping better track of changes in the voxel label. Using the volumetric structure which we are able to generate.
This is currently not really used given that we only keep track of the posterior for each voxel.
In 3D the voxels or rendered mesh it seems to be easier to use geometric information  to cluster objects. 


