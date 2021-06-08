# Overview:



```generate_maps```: Starts the mapping process of multiple scenes. These can be simply defined in the exp file with the correct identifier. Produces the resulting mesh + serialized voxel information

```generate_labels```: Reprojects the map back to the image plane for the full camera trajektory. Produces .png with class probabilities for each frame in the sequence and automatically creates the .png with the ```_reprojected``` identifier.

```dl_mock```: Allows publish information for Kimera-Semantics for ScanNet and pre-inferenced semantic labels.


