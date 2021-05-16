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