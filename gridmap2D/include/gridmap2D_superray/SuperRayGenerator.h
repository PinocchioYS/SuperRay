/*
* Copyright(c) 2016, Youngsun Kwon, Donghyuk Kim, and Sung-eui Yoon, KAIST
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met :
*
*     * Redistributions of source code must retain the above copyright notice, this
*       list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright notice,
*       this list of conditions and the following disclaimer in the documentation
*       and / or other materials provided with the distribution.
*     * Neither the name of SuperRay nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef GRIDMAP2D_SUPERRAY_SUPERRAY_GENERATOR_H
#define GRIDMAP2D_SUPERRAY_SUPERRAY_GENERATOR_H

#include <gridmap2D/Grid2DKey.h>
#include <gridmap2D/Pointcloud.h>
#include <gridmap2D_superray/SuperRayCloud.h>

/* Libc++ does not implement the TR1 namespace, all c++11 related functionality
 * is instead implemented in the std namespace.
 */
/*#if defined(__GNUC__) && ! defined(_LIBCPP_VERSION)
namespace unordered_ns = std::tr1;
#else
namespace unordered_ns = std;
#endif*/

#ifdef _OPENMP
#include <omp.h>
#endif

namespace gridmap2D{
	class SuperRayGenerator{
	public:
		SuperRayGenerator(const double _resolution, const unsigned int _grid_max_val, const int _threshold = 0);
		~SuperRayGenerator() {};
	
		void GenerateSuperRay(const Pointcloud& _pc, const point2d& _origin, SuperRayCloud& _srcloud);

	protected:
		struct VoxelInfo;
		struct Axis2D;

		point2d		originW;	// origin point in World Space
		Grid2DKey	originKey;	// origin key
		
		// constants for generating super rays
		double			RESOLUTION;			// resolution
		double			RESOLUTION_FACTOR;	// 1.0 / resolution
		unsigned int	GRID_MAX_VAL;		// offset
		unsigned int	THRESHOLD;			// threshold for limiting to generate super rays for each voxel

		// Functions for generating super rays
		void GenerateSuperRay(const point2d_collection& _pointlist, SuperRayCloud& _srcloud);
		void GenerateSuperRay2D(const point2d_collection& _pointlist, Axis2D& _axis, VoxelInfo& _voxelinfo, SuperRayCloud& _srcloud);

		// Function for generating mapping line in 2-D
		double GenerateMappingLine(VoxelInfo& _voxelinfo, const unsigned int& _axisX, const unsigned int& _axisY, std::vector<double>& _mappingPlane);

		// Utility functions
		typedef unordered_ns::unordered_map<Grid2DKey, std::vector<point2d>, Grid2DKey::KeyHash> Vexelized_Pointclouds;
		void ComputeAxis(const point2d& _min, const point2d& _max, Axis2D& _axis);

		// Re-implmentation for Key / coordinate conversion functions
		inline Grid2DKey coordToKey(const point2d& coord) const {
			return Grid2DKey(coordToKey(coord(0)), coordToKey(coord(1)));
		}
		inline key_type coordToKey(double coordinate) const {
			return ((int)floor(RESOLUTION_FACTOR * coordinate)) + GRID_MAX_VAL;
		}

		// Structures that represents the traversal information
		struct VoxelInfo{
			VoxelInfo(void) {};
			// Voxel Info.
			point2d minW;		// min position of voxel
			point2d maxW;		// max position of voxel
			Grid2DKey voxelKey;	// key of voxel
		};

		struct Axis2D{
			Axis2D(void) : axisU(0), axisV(1) {};

			unsigned int axisU;	// Nearest Axis
			unsigned int axisV;	// Farest Axis
		};
	};
}

#endif
