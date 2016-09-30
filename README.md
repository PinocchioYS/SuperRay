SuperRay - High performance method for updating occupancy maps with point clouds
================================================================================

http://sglab.kaist.ac.kr/projects/SuperRay

SuperRay is a library for efficiently updating occupancy map representation with point clouds.
The update approach based on super ray shows high performance without compromising representation accuracy.
The implementation of our library is based on [OctoMap library](https://github.com/Octomap/octomap).
You can see the detailed information of super ray based updates at [here](http://sglab.kaist.ac.kr/SuperRay).

License
-------
SuperRay: [NEW BSD License](superray/LICENSE.txt)
OctoMap: [New BSD License](octomap/LICENSE.txt)

BUILD
-----
You can build the SuperRay and OctoMap library together with CMake in the top-level directory.
The implementation of recent SuperRay 1.1.0 depends on OctoMap 1.8.0.
E.g. for compiling the library, run:

	cd SuperRay-master
	mkdir build
	cd build
	cmake ..
	make

Binaries and libs will end up in the directories 'bin' and 'lib' of the top-level directory where you started the build.

See [octomap README](octomap/README.md) for further details and hints on compiling.
Authors of OctoMap library describe how to compile the libraries on various platforms and IDEs.

We tested compiling on Ubuntu14.04 and over MSVC2010.
If you have any problem or issue, notice it at [here](https://github.com/PinocchioYS/SuperRay/issues).