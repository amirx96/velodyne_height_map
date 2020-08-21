# Velodyne_Height_Map with Occupancy Grid Conversion

## Intro

![Image of costmap](https://amirdarwesh.com/assets/img/2020-08-19/occupancy_grid_velodyne.png)

The algorithm is a fork from jack-oquin / velodyne_height_map where obstacles are detected in a pointcloud via a height map. My fork just adds another node that publishes the obstacle pointcloud as an ROS occupancy grid map.

Note that this should work on any type of 3D LIDAR (such as Ouster 16), and the intensity value is not required. I am currently using this node to generate an occupancy grid map in CARLA.

## Running 

Launch the original velodyne height map node first via:

`rosrun velodyne_height_map heightmap_node`

Launch the heightmap to costmap node:

`rosrun velodyne_height_map heightmap_costmap_node`

## Subscribed Topics

* /velodyne_points

## Published Topics

* /velodyne_obstacles
* /velodyne_clear
* /costmap
