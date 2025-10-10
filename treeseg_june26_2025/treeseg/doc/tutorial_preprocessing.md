# Preprocessing

## The larger-area point cloud

The primary input to treeseg is some larger-area point cloud collected in a forest scene (e.g., in this tutorial, the point cloud representing the 1 ha tropical rainforest plot in French Guiana). It is expected that this point cloud is stored in binary or ASCII [PCD format](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcd_file_format.html). By default, the expected point fields are the Cartesian coordinates (x, y, z) of each point, but additional fields can be included (as described in the following subsection). It is also expected that the larger-area point cloud is broken into N tiles (N<sub>min</sub> = 1), using the following strict naming convention:

```
mycloud.tile.0.pcd
mycloud.tile.1.pcd
...
mycloud.tile.N.pcd
```

Where 'mycloud' can be replaced with some meaningful identifier of the larger-area point cloud. Depending on computing environment and resources, it will often be necessary for each tile to have a maximum size of a few gigabytes, to circumvent issues that potentially arise from requiring large contiguous blocks of unoccupied RAM to be available when loading large point clouds.

## Parsing additional point fields

treeseg implements a custom point type, `PointTreeseg`, to provide the user with flexibility over the point fields. By default PointTreeseg = [pcl::PointXYZ](https://pointclouds.org/documentation/structpcl_1_1_point_x_y_z.html). However, the definition of [PointTreeseg](../include/treeseg_pointtype.h#L33) can be modified if the user wishes to include other fields (e.g., reflectance).  

An example of how this can be implemented is provided by the preprocessor macro [XYZRRDRS](../include/treeseg_pointtype.h#L29). If this macro is defined as true (default = false) and treeseg recompiled, then the `PointTreeseg` point fields become: x (m), y (m), z (m), range (m), reflectance (dB), deviation (#), return number (#) and scan number (#). These additional fields will then be preserved throughout the various subsequent processing steps.

## RXP to PCD conversion

In this tutorial we only consider preprocessing lidar data stored in RIEGL's RXP data stream format, to the abovedescribed expected tiled PCD format. However, various open-source tools are available for converting lidar data in other formats (e.g., LAS).

The raw lidar data downloaded during the previous step (i.e., [option 1](tutorial_overview.md#Data)) are preprocessed using the `rxp2pcd` executable. The overall x-y limits of the larger-area point cloud are defined by the coordinates of a bounding box, which here, we want dictated by the plot boundaries, which can be approximated from the scan locations via the transformation matrices in `./treeseg_tutorial/data/matrix/` using `plotcoords`:

```
cd ../processing/;
plotcoords ../data/matrix/ > NOU11.coords.dat;
```

Whereby the resulting ASCII file `NOU11.coords.dat` contains:
 
```
-101.261 4.48606 -104.789 0
```

I.e., the minimum and maximum coordinates of the 121 scan locations (xmin, xmax, ymin, ymax), in meters.

`rxp2pcd` can then be called as:

```
rxp2pcd ../data/ NOU11.coords.dat 25 15 NOU11
```

Where the inputs are: 1) the top directory containing the lidar data; 2) the coordinates of the plot boundaries (note: the dimensions of the bounding box are increased by 20 m in each direction to ensure data relating to the trees of interest are not truncated (e.g., the crowns of large trees at the edges of the plot); 3) the area of each output tile (m<sup>2</sup>); 4) the maximum permitted value of point deviation (a proxy for range estimate goodness that is used to remove noisy returns); and 5) the larger-area point cloud identifier. 

This command will output the larger-area point cloud into 870 binary PCD tiles: `NOU11.tile.0.pcd` to `NOU11.tile.869.pcd`. These tiles can then be viewed, e.g., using `pcl_viewer`:

```
pcl_viewer NOU11.tile.342.pcd
```

## [Next: Downsampling](tutorial_downsample.md)
