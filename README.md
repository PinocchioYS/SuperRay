SuperRay - High performance method for updating occupancy maps with point clouds
================================================================================

SuperRay is a library for efficiently updating the occupancy map representation with point clouds.
The update method based on both super rays and culling region shows high performance without compromising representation accuracy.
Our library provides four different types of the occupancy maps - octree, quadtree, grid3D, grid2D - of which the implementations
are based on [OctoMap library](https://github.com/Octomap/octomap).
You can see the technical details of super rays and culling region based updates at [here](http://sgvr.kaist.ac.kr/~yskwon/papers/tro19-superray-cullingregion/).

License
-------
* SuperRay: [New BSD License](LICENSE.txt)
* OctoMap: [New BSD License](octomap/LICENSE.txt)

BUILD
-----
You can build the SuperRay library by using the CMake in the top-level directory.
(The implementation of recent SuperRay library depends on OctoMap 1.9.0.)
E.g. for compiling the library, run:

	cd SuperRay-master
	mkdir build
	cd build
	cmake ..
	make

Binaries and libs will end up in the directories 'bin' and 'lib' of the top-level directory where you started the build.

See [octomap README](octomap/README.md) for further details and hints on compiling.
Authors of OctoMap library describe how to compile the libraries on various platforms and IDEs.

We tested compiling on Ubuntu16.04 and over MSVC2010.
If you have any problem or issue, notice it at [here](https://github.com/PinocchioYS/SuperRay/issues).