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

#ifndef OCTOMAP_SUPERRAY_SUPERRAY_GENERATOR_H
#define OCTOMAP_SUPERRAY_SUPERRAY_GENERATOR_H

#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <octomap/Pointcloud.h>
#include <octomap_superray/SuperRayCloud.h>

#ifdef _OPENMP
#include <omp.h>
#pragma omp declare reduction (merge : std::vector<octomap::SuperRay> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#endif

namespace octomap{
	class SuperRayGenerator{
	public:
		SuperRayGenerator(const double _resolution, const unsigned int _tree_max_val, const int _threshold = 0);
		~SuperRayGenerator() {};
	
		void GenerateSuperRay(const octomap::Pointcloud& _pc, const octomap::point3d& _origin, SuperRayCloud& _srcloud);

	protected:
		struct VoxelInfo;
		struct Axis3D;

		octomap::point3d	originW;	// origin point in World Space
		octomap::OcTreeKey	originKey;	// origin key
		
		// constants for generating super rays
		double			RESOLUTION;			// resolution
		double			RESOLUTION_FACTOR;	// 1.0 / resolution
		unsigned int	TREE_MAX_VAL;		// offset
		unsigned int	THRESHOLD;			// threshold for limiting to generate super rays for each voxel

		// Functions for generating super rays
        void GenerateSuperRay(const point3d_collection& _pointlist, std::vector<SuperRay>& _srcloud);
        void GenerateSuperRay2D(const point3d_collection& _pointlist, Axis3D& _axis, VoxelInfo& _voxelinfo, std::vector<SuperRay>& _srcloud);
        void GenerateSuperRay3D(const point3d_collection& _pointlist, Axis3D& _axis, VoxelInfo& _voxelinfo, std::vector<SuperRay>& _srcloud);

		// Function for generating mapping line in 2-D
		double GenerateMappingLine(VoxelInfo& _voxelinfo, const unsigned int& _axisX, const unsigned int& _axisY, std::vector<double>& _mappingPlane);

		// Utility functions
		typedef unordered_ns::unordered_map<octomap::OcTreeKey, std::vector<octomap::point3d>, octomap::OcTreeKey::KeyHash> Voxelized_Pointclouds;
		void ComputeAxis(const octomap::point3d& _min, const octomap::point3d& _max, Axis3D& _axis);

		// Re-implmentation for Key / coordinate conversion functions
		inline octomap::OcTreeKey coordToKey(const octomap::point3d& coord) const {
			return octomap::OcTreeKey(coordToKey(coord(0)), coordToKey(coord(1)), coordToKey(coord(2)));
		}
		inline octomap::key_type coordToKey(double coordinate) const {
			return ((int)floor(RESOLUTION_FACTOR * coordinate)) + TREE_MAX_VAL;
		}

		// Structures that represents the traversal information
		struct VoxelInfo{
			VoxelInfo(void) {};
			// Voxel Info.
			octomap::point3d minW;		// min position of voxel
			octomap::point3d maxW;		// max position of voxel
			octomap::OcTreeKey voxelKey;	// key of voxel
		};

		struct Axis3D{
			Axis3D(void) : axisU(0), axisV(1), axisK(2) {};

			unsigned int axisU;	// Nearest Axis
			unsigned int axisV;	// 
			unsigned int axisK;	// Farthest Axis
		};
	};
}

#endif
