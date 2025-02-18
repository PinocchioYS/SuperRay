SuperRay - High performance method for updating occupancy maps with point clouds
================================================================================

SuperRay is a library for efficiently updating the occupancy map representation with point clouds.
The update method based on both super rays and culling region shows high performance without compromising representation accuracy.
Our library provides four different types of the occupancy maps - octree, quadtree, grid3d, grid2d - of which the implementations are based on [OctoMap library](https://github.com/Octomap/octomap).
You can see the technical details of super rays and culling region based updates at [here](http://sgvr.kaist.ac.kr/~yskwon/papers/tro19-superray-cullingregion/).

BUILD & INSTALL
-----
You can build the SuperRay library by using the CMake in the top-level directory.
Note that SuperRay-OctoMap requires the pre-installed OctoMap library.
E.g. for compiling the library, run:

	git clone https://github.com/PinocchioYS/SuperRay.git
	cd SuperRay
	mkdir build
	cd build
	cmake ..
	make

Binaries and libs will end up in the directories 'bin' and 'lib' of the top-level directory where you started the build.

You can use our library in your project by setting its include directories and lib files manually.
But we suggest using the "find_package()" command in CMake as a more convenient way after installing this library.
For installing it, run:

	git clone https://github.com/PinocchioYS/SuperRay.git
	cd SuperRay
	mkdir build
	cd build
	cmake ..
	make install

Then you can link each type of occupancy map through the command of CMake, for example:

    find_package(superray-octomap REQUIRED)
    find_package(superray-quadmap REQUIRED)
    find_package(superray-gridmap3d REQUIRED)
    find_package(superray-gridmap2d REQUIRED)

See [octomap README](octomap/README.md) for further details and hints on compiling.
Authors of OctoMap library describe how to compile the libraries on various platforms and IDEs.

EXAMPLE USING THIS LIBRARY IN ROS/ROS2
--------------------------------
We provide a ROS package that subscribes a pointcloud and updates an occupancy map in real time.
Refer the [realtime_occupancy_mapping](https://github.com/PinocchioYS/realtime_occupancy_mapping) project.

VISUALIZATION
-------------
You can visualize the occupancy maps in RViz on ROS/ROS2.
Refer the [occupancy_map_visualizer](https://github.com/PinocchioYS/occupancy_map_visualizer) project.

If you have any problem or issue, notice it at [here](https://github.com/PinocchioYS/SuperRay/issues).

License
-------
* SuperRay: [New BSD License](LICENSE)
* OctoMap: [New BSD License](octomap/LICENSE.txt)