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

#include <gridmap3D_superray/SuperRayGenerator.h>

#include <algorithm>
#include <cfloat>
#include <map>

namespace gridmap3D{
	SuperRayGenerator::SuperRayGenerator(const double _resolution, const unsigned int _tree_max_val, const int _threshold) {
		// Initialize constants
		RESOLUTION = _resolution;
		RESOLUTION_FACTOR = 1.0 / _resolution;
		GRID_MAX_VAL = _tree_max_val;
		THRESHOLD = _threshold;
	}

	void SuperRayGenerator::GenerateSuperRay(const Pointcloud& _pc, const point3d& _origin, SuperRayCloud& _srcloud) {
		originW = _origin;
		originKey = coordToKey(_origin);

		// Voxelize point clouds
		Vexelized_Pointclouds voxels;
		for (unsigned int i = 0; i < _pc.size(); i++){
			voxels[coordToKey(_pc[i])].push_back(_pc[i]);
		}

		_srcloud.origin = _origin;
	#ifdef _OPENMP
		std::vector< std::vector<point3d>* > pointlistvector;
		Vexelized_Pointclouds::iterator it;
		for (it = voxels.begin(); it != voxels.end(); ++it){
			pointlistvector.push_back(&(it->second));
		}

	#pragma omp parallel for
		for (int i = 0; i < (int)pointlistvector.size(); i++){
			std::vector<point3d>& pointlist = *(pointlistvector[i]);

			// Skip to generate super rays -> insert all rays
			if (pointlist.size() < THRESHOLD){
	#pragma omp critical
				{
					for (unsigned int j = 0; j < pointlist.size(); ++j)
						_srcloud.push_back(pointlist[j], 1);
				}
				continue;
			}

			// Generate super rays from point clouds
			GenerateSuperRay(pointlist, _srcloud);
		}
	#else
		Vexelized_Pointclouds::iterator it;
		for (it = voxels.begin(); it != voxels.end(); ++it){
			std::vector<point3d>& pointlist = it->second;

			// Skip to generate super rays -> insert all rays
			if (pointlist.size() < THRESHOLD){
				for (unsigned int j = 0; j < pointlist.size(); ++j)
					_srcloud.push_back(pointlist[j], 1);
				continue;
			}

			// Generate super rays from point clouds
			GenerateSuperRay(pointlist, _srcloud);
		}
	#endif
	}

	void SuperRayGenerator::GenerateSuperRay(const point3d_collection& _pointlist, SuperRayCloud& _srcloud) {
		// 0. Initialize vertices of voxel
		VoxelInfo voxelinfo;
		voxelinfo.voxelKey = coordToKey(_pointlist[0]);
		voxelinfo.minW.x() = (float)((voxelinfo.voxelKey.k[0] - (unsigned short)GRID_MAX_VAL) * RESOLUTION);	// Min X of voxel
		voxelinfo.minW.y() = (float)((voxelinfo.voxelKey.k[1] - (unsigned short)GRID_MAX_VAL) * RESOLUTION);	// Min Y of voxel
		voxelinfo.minW.z() = (float)((voxelinfo.voxelKey.k[2] - (unsigned short)GRID_MAX_VAL) * RESOLUTION);	// Min Z of voxel
		voxelinfo.maxW.x() = (float)(voxelinfo.minW.x() + RESOLUTION);
		voxelinfo.maxW.y() = (float)(voxelinfo.minW.y() + RESOLUTION);
		voxelinfo.maxW.z() = (float)(voxelinfo.minW.z() + RESOLUTION);

		// 1. Compute the traversal axis for finding all grid points in a frustum efficiently
		Axis3D axis;
		ComputeAxis(voxelinfo.minW, voxelinfo.maxW, axis);

		// 2. Generate super rays in 3-D
		GenerateSuperRay3D(_pointlist, axis, voxelinfo, _srcloud);
	}

	void SuperRayGenerator::GenerateSuperRay2D(const point3d_collection& _pointlist, Axis3D& _axis, VoxelInfo& _voxelinfo, SuperRayCloud& _srcloud) {
		// 0. Initialize Constants - Re-mapping two axes to X and Y axis
		const unsigned int AXISX = _axis.axisV;		// Traversal Axis
		const unsigned int AXISY = _axis.axisK;		// Mapping Axis
		Grid3DKey& voxelKey = _voxelinfo.voxelKey;	// Key Coordinate
		point3d originT(originW(AXISX), originW(AXISY), 0);

		// Special case - Only one super ray
		if (originKey.k[AXISX] == voxelKey.k[AXISX]){
	#ifdef _OPENMP
	#pragma omp critical
	#endif
			{
				_srcloud.push_back(_pointlist[0], (int)_pointlist.size());
			}
			return;
		}

		// 1. Generate one mapping line in 2-D
		std::vector<double> mappingPlaneY;
		double mappingX = GenerateMappingLine(_voxelinfo, AXISX, AXISY, mappingPlaneY);

		// 2. Generate super rays using the mapping line
		std::map<unsigned int, SuperRay> superrays;
		double pointX, pointY, mappingPointY;
		for (unsigned int i = 0; i < _pointlist.size(); ++i){
			pointX = _pointlist[i](AXISX);
			pointY = _pointlist[i](AXISY);

			// Find a segment where a point is mapped
			unsigned int idx = 0;
			if (mappingPlaneY.size() != 1){
				// Project a point onto the mapping line
				mappingPointY = (pointY - originT.y()) * (mappingX - originT.x()) / (pointX - originT.x()) + originT.y();
				// Binary Search
				std::vector<double>::iterator it = std::lower_bound(mappingPlaneY.begin(), mappingPlaneY.end(), mappingPointY);
				idx = (unsigned int)std::distance(mappingPlaneY.begin(), it);
			}
			else{	// XYspace.size() == 1
				idx = 0;
			}

			// A point is mapped to the same segment where the other point is mapped
			if (superrays.find(idx) != superrays.end()){
				superrays[idx].w++;		// Increase weight of a super ray
			}
			else{
				SuperRay sr(_pointlist[i], 1);
				superrays[idx] = sr;	// Create a new super ray
			}
		}

		// 3. Push back the generated super rays into super ray cloud
	#ifdef _OPENMP
	#pragma omp critical
	#endif
		{
			std::map<unsigned int, SuperRay>::iterator rayIt;
			for (rayIt = superrays.begin(); rayIt != superrays.end(); rayIt++){
				_srcloud.push_back(rayIt->second);
			}
		}
	}

	void SuperRayGenerator::GenerateSuperRay3D(const point3d_collection& _pointlist, Axis3D& _axis, VoxelInfo& _voxelinfo, SuperRayCloud& _srcloud) {
		// Special case - We need only two axis for generating super rays in 3-D.
		if (originKey.k[_axis.axisU] == _voxelinfo.voxelKey.k[_axis.axisU]) {
			GenerateSuperRay2D(_pointlist, _axis, _voxelinfo, _srcloud);
			return;
		}

		// 0. Initialize Constants - Re-mapping 
		const unsigned int& axisX = _axis.axisU;
		const unsigned int& axisY = _axis.axisV;
		const unsigned int& axisZ = _axis.axisK;	// Traversal Direction
		point3d originT(originW(axisX), originW(axisY), originW(axisZ));

		// 1. Generate mapping lines of three 2-D sub spaces.
		std::vector<double> mappingPlaneXY, mappingPlaneZX, mappingPlaneZY;
		double mappingXY = GenerateMappingLine(_voxelinfo, axisX, axisY, mappingPlaneXY);
		double mappingZX = GenerateMappingLine(_voxelinfo, axisZ, axisX, mappingPlaneZX);
		double mappingZY = GenerateMappingLine(_voxelinfo, axisZ, axisY, mappingPlaneZY);

		// 2. Inserting
		std::map<unsigned int, SuperRay> superrays;
		double pointX, pointY, pointZ;
		for (unsigned int i = 0; i < _pointlist.size(); i++){
			pointX = _pointlist[i](axisX);	// Traversal Cooridnate
			pointY = _pointlist[i](axisY);	// Traversal Cooridnate
			pointZ = _pointlist[i](axisZ);	// Traversal Cooridnate

			// Find a segment where a point is mapped
			unsigned int idx[3];
			// X-Y plane
			if (mappingPlaneXY.size() != 1){
				// Project a point onto the mapping line of X-Y plane
				double mappingPointY = (pointY - originT.y()) * (mappingXY - originT.x()) / (pointX - originT.x()) + originT.y();
				// Binary Search
		  std::vector<double>::iterator it = std::lower_bound(mappingPlaneXY.begin(), mappingPlaneXY.end(), mappingPointY);
		  idx[0] = (unsigned int)std::distance(mappingPlaneXY.begin(), it);
			}
			else{	// XYspace.size() == 1
				idx[0] = 0;
			}
			// Z-X plane
			if (mappingPlaneZX.size() != 1){
				// Project a point onto the mapping line of Z-X plane
				double mappingPointX = (pointX - originT.x()) * (mappingZX - originT.z()) / (pointZ - originT.z()) + originT.x();
				// Binary Search
		  std::vector<double>::iterator it = std::lower_bound(mappingPlaneZX.begin(), mappingPlaneZX.end(), mappingPointX);
		  idx[1] = (unsigned int)std::distance(mappingPlaneZX.begin(), it);
			}
			else{	// XYspace.size() == 1
				idx[1] = 0;
			}
			// Z-Y plane
			if (mappingPlaneZY.size() != 1){
				// Project a point onto the mapping line of Z-Y plane
				double mappingPointY = (pointY - originT.y()) * (mappingZY - originT.z()) / (pointZ - originT.z()) + originT.y();
				// Binary Search
		  std::vector<double>::iterator it = std::lower_bound(mappingPlaneZY.begin(), mappingPlaneZY.end(), mappingPointY);
		  idx[2] = (unsigned int)std::distance(mappingPlaneZY.begin(), it);
			}
			else{	// XYspace.size() == 1
				idx[2] = 0;
			}
			unsigned int index = idx[0] * (unsigned int)mappingPlaneZX.size() * (unsigned int)mappingPlaneZY.size() + idx[1] * (unsigned int)mappingPlaneZY.size() + idx[2];

			// A point is mapped to the same segment where the other point is mapped
			if (superrays.find(index) != superrays.end()){
				superrays[index].w++;	// Increase weight of a super ray
			}
			else{
				SuperRay sr(_pointlist[i], 1);
				superrays[index] = sr;	// Create a new super ray
			}
		}

		// 3. Push back the generated super rays into super ray cloud
	#ifdef _OPENMP
	#pragma omp critical
	#endif
		{
			std::map<unsigned int, SuperRay>::iterator rayIt;
			for (rayIt = superrays.begin(); rayIt != superrays.end(); rayIt++){
				_srcloud.push_back(rayIt->second);
			}
		}
	}

	double SuperRayGenerator::GenerateMappingLine(VoxelInfo& _voxelinfo, const unsigned int& _axisX, const unsigned int& _axisY, std::vector<double>& _mappingPlane) {
		// Find all grid points in a frustum, and then generate a mapping line
		// If you want the details, see "Ray Tracing Animated Scenes using Coherent Grid Traversal" by Ingo Wald et al.

		// 0. Initialize information used in frustum traversal
		Grid3DKey& voxelKey = _voxelinfo.voxelKey;	// Key Coordinate
		point3d& minW = _voxelinfo.minW;
		point3d& maxW = _voxelinfo.maxW;
		point3d originT(originW(_axisX), originW(_axisY), 0);	// Traversal Cooridnate
		point3d verticesT[4];									// Traversal Coordinate
		verticesT[0].x() = minW(_axisX);	verticesT[0].y() = minW(_axisY);	verticesT[0].z() = (float)0.0;
		verticesT[1].x() = minW(_axisX);	verticesT[1].y() = maxW(_axisY);	verticesT[1].z() = (float)0.0;
		verticesT[2].x() = maxW(_axisX);	verticesT[2].y() = minW(_axisY);	verticesT[2].z() = (float)0.0;
		verticesT[3].x() = maxW(_axisX);	verticesT[3].y() = maxW(_axisY);	verticesT[3].z() = (float)0.0;
		// Traversal directioin
		int dir = 0;
		if (abs(originT.x() - verticesT[0].x()) < abs(originT.x() - verticesT[3].x()))	dir = 1;
		else																			dir = -1;
		// Start and goal line of traversal axis
		int curXKey, mappingXKey;
		double curX, mappingX;
		if (dir == 1){
			curXKey = originKey.k[_axisX] + 1;
			curX = (curXKey - (int)GRID_MAX_VAL) * RESOLUTION;
			mappingXKey = voxelKey.k[_axisX];
			mappingX = verticesT[0].x();
		}
		else{
			curXKey = originKey.k[_axisX];
			curX = (curXKey - (int)GRID_MAX_VAL) * RESOLUTION;
			mappingXKey = voxelKey.k[_axisX] - 1;
			mappingX = verticesT[3].x();
		}
		// Compute min & max of frustum traversal range
		double min = DBL_MAX, max = -DBL_MAX;
		for (unsigned int i = 0; i < 4; i++){
			double y = (verticesT[i].y() - originT.y()) * (mappingX - originT.x()) / (verticesT[i].x() - originT.x()) + originT.y();
			if (y < min)	min = y;
			if (y > max)	max = y;
		}

		// 1. Traverse grids along traversal axis
		_mappingPlane.push_back(max);
		while (dir * (mappingXKey - curXKey) >= 0){
			// Find a ranage that the frustum encounters a slice
			double constD = (curX - originT.x()) / (mappingX - originT.x());
			double minY = (min - originT.y()) * constD + originT.y();
			double maxY = (max - originT.y()) * constD + originT.y();

			// Insert grid points into mapping line 
			int minYKey = coordToKey(minY) + 1;
			int maxYKey = coordToKey(maxY);
			for (int y = minYKey; y <= maxYKey; y++){
				double curY = (y - (int)GRID_MAX_VAL) * RESOLUTION;
				double projectionY = (curY - originT.y()) / constD + originT.y();
				_mappingPlane.push_back(projectionY);
			}

			// Next line
			curXKey = curXKey + dir;
			curX = curX + dir * RESOLUTION;
		}

		// 2. Remove duplicated points in the mapping line
		_mappingPlane.erase(unique(_mappingPlane.begin(), _mappingPlane.end()), _mappingPlane.end());
		std::sort(_mappingPlane.begin(), _mappingPlane.end());

		return mappingX;
	}

	void SuperRayGenerator::ComputeAxis(const point3d& _min, const point3d& _max, Axis3D& _axis) {
		// Compute traveral axis for generating a mapping lines efficiently
		// If you want the details, see "Ray Tracing Animated Scenes using Coherent Grid Traversal" by Ingo Wald et al.

		double nearDist[3];
		for (unsigned int i = 0; i < 3; i++){
			if (abs(originW(i) - _min(i)) < abs(originW(i) - _max(i)))	nearDist[i] = abs(originW(i) - _min(i));
			else 														nearDist[i] = abs(originW(i) - _max(i));
		}

		if (nearDist[0] < nearDist[1]){
			if (nearDist[0] < nearDist[2]){
				_axis.axisU = 0;
				if (nearDist[1] < nearDist[2]){
					_axis.axisV = 1;
					_axis.axisK = 2;
				}
				else{
					_axis.axisV = 2;
					_axis.axisK = 1;
				}
			}
			else{
				_axis.axisU = 2;
				_axis.axisV = 0;
				_axis.axisK = 1;
			}
		}
		else{
			if (nearDist[1] < nearDist[2]){
				_axis.axisU = 1;
				if (nearDist[0] < nearDist[2]){
					_axis.axisV = 0;
					_axis.axisK = 2;
				}
				else{
					_axis.axisV = 2;
					_axis.axisK = 0;
				}
			}
			else{
				_axis.axisU = 2;
				_axis.axisV = 1;
				_axis.axisK = 0;
			}
		}
	}
}
