# transform_surfels
This package just transforms surfels from the convex_segmentation package in the map coordinate system

## Usage

To transform the surfel map and all the convex segments in one folder, run the following command:
```
rosrun transform_surfels transform_sweep path/to/my/room.xml
```
This will create new files `path/to/my/transformed_surfel_map.pcd` and `path/to/my/convex_segments/transformed_segment*.pcd`. The transformed point clouds are in the map coordinate system.
