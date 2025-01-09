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

#include <superray_gridmap2d/SuperRayGenerator.h>

#include <algorithm>
#include <cfloat>
#include <map>

namespace gridmap2D{
	SuperRayGenerator::SuperRayGenerator(const double _resolution, const unsigned int _grid_max_val, const int _threshold) {
		// Initialize constants
		RESOLUTION = _resolution;
		RESOLUTION_FACTOR = 1.0 / _resolution;
		GRID_MAX_VAL = _grid_max_val;
		THRESHOLD = _threshold;
	}

	void SuperRayGenerator::GenerateSuperRay(const Pointcloud& _pc, const point2d& _origin, SuperRayCloud& _srcloud) {
		originW = _origin;
		originKey = coordToKey(_origin);

		// Voxelize point clouds
		Voxelized_Pointclouds voxels;
		for (unsigned int i = 0; i < _pc.size(); i++){
			voxels[coordToKey(_pc[i])].push_back(_pc[i]);
		}

		_srcloud.origin = _origin;
		std::vector<SuperRay>& superrays = _srcloud.superrays;

#ifdef _OPENMP
		std::vector< std::vector<point2d>* > pointlistvector;
		for (Voxelized_Pointclouds::iterator it = voxels.begin(); it != voxels.end(); ++it){
			pointlistvector.push_back(&(it->second));
		}

#pragma omp parallel for schedule(guided) reduction(merge : superrays)
		for (int i = 0; i < (int)pointlistvector.size(); i++){
			std::vector<point2d>& pointlist = *(pointlistvector[i]);

			// Skip to generate super rays -> insert all rays
			if (pointlist.size() < THRESHOLD){
                for (unsigned int j = 0; j < pointlist.size(); ++j)
                    superrays.push_back(SuperRay(pointlist[j], 1));
				continue;
			}

			// Generate super rays from point clouds
			GenerateSuperRay(pointlist, superrays);
		}
	#else
		for (Voxelized_Pointclouds::iterator it = voxels.begin(); it != voxels.end(); ++it){
			std::vector<point2d>& pointlist = it->second;

			// Skip to generate super rays -> insert all rays
			if (pointlist.size() < THRESHOLD){
				for (unsigned int j = 0; j < pointlist.size(); ++j)
					superrays.push_back(SuperRay(pointlist[j], 1));
				continue;
			}

			// Generate super rays from point clouds
			GenerateSuperRay(pointlist, superrays);
		}
	#endif
	}

	void SuperRayGenerator::GenerateSuperRay(const point2d_collection& _pointlist, std::vector<SuperRay>& _srcloud) {
		// 0. Initialize vertices of pixel
		PixelInfo pixelinfo;
		pixelinfo.pixelKey = coordToKey(_pointlist[0]);
		pixelinfo.minW.x() = (float)((pixelinfo.pixelKey.k[0] - (unsigned short)GRID_MAX_VAL) * RESOLUTION);	// Min X of pixel
		pixelinfo.minW.y() = (float)((pixelinfo.pixelKey.k[1] - (unsigned short)GRID_MAX_VAL) * RESOLUTION);	// Min Y of pixel
		pixelinfo.maxW.x() = (float)(pixelinfo.minW.x() + RESOLUTION);
		pixelinfo.maxW.y() = (float)(pixelinfo.minW.y() + RESOLUTION);

		// 1. Compute the traversal axis for finding all grid points in a frustum efficiently
		Axis2D axis;
		ComputeAxis(pixelinfo.minW, pixelinfo.maxW, axis);

		// 2. Generate super rays in 2-D
		GenerateSuperRay2D(_pointlist, axis, pixelinfo, _srcloud);
	}

	void SuperRayGenerator::GenerateSuperRay2D(const point2d_collection& _pointlist, Axis2D& _axis, PixelInfo& _pixelinfo, std::vector<SuperRay>& _srcloud) {
		// 0. Initialize Constants - Re-mapping two axes to X and Y axis
		const unsigned int AXISX = _axis.axisU;		// Traversal Axis
		const unsigned int AXISY = _axis.axisV;		// Mapping Axis
		Grid2DKey& pixelKey = _pixelinfo.pixelKey;	// Key Coordinate
		point2d originT(originW(AXISX), originW(AXISY));

		// Special case - Only one super ray
		if (originKey.k[AXISX] == pixelKey.k[AXISX]){
            _srcloud.push_back(SuperRay(_pointlist[0], (int)_pointlist.size()));
			return;
		}

		// 1. Generate one mapping line in 2-D
		std::vector<double> mappingPlaneY;
		double mappingX = GenerateMappingLine(_pixelinfo, AXISX, AXISY, mappingPlaneY);

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
        std::map<unsigned int, SuperRay>::iterator rayIt;
        for (rayIt = superrays.begin(); rayIt != superrays.end(); rayIt++){
            _srcloud.push_back(rayIt->second);
        }
	}

	double SuperRayGenerator::GenerateMappingLine(PixelInfo& _pixelinfo, const unsigned int& _axisX, const unsigned int& _axisY, std::vector<double>& _mappingPlane) {
		// Find all grid points in a frustum, and then generate a mapping line
		// If you want the details, see "Ray Tracing Animated Scenes using Coherent Grid Traversal" by Ingo Wald et al.

		// 0. Initialize information used in frustum traversal
		Grid2DKey& pixelKey = _pixelinfo.pixelKey;	// Key Coordinate
		point2d& minW = _pixelinfo.minW;
		point2d& maxW = _pixelinfo.maxW;
		point2d originT(originW(_axisX), originW(_axisY));	// Traversal Cooridnate
		point2d verticesT[4];								// Traversal Coordinate
		verticesT[0].x() = minW(_axisX);	verticesT[0].y() = minW(_axisY);
		verticesT[1].x() = minW(_axisX);	verticesT[1].y() = maxW(_axisY);
		verticesT[2].x() = maxW(_axisX);	verticesT[2].y() = minW(_axisY);
		verticesT[3].x() = maxW(_axisX);	verticesT[3].y() = maxW(_axisY);
		// Traversal directioin
		int dir = 0;
		if (fabs(originT.x() - verticesT[0].x()) < fabs(originT.x() - verticesT[3].x()))	dir = 1;
		else																			    dir = -1;
		// Start and goal line of traversal axis
		int curXKey, mappingXKey;
		double curX, mappingX;
		if (dir == 1){
			curXKey = originKey.k[_axisX] + 1;
			curX = (curXKey - (int)GRID_MAX_VAL) * RESOLUTION;
			mappingXKey = pixelKey.k[_axisX];
			mappingX = verticesT[0].x();
		}
		else{
			curXKey = originKey.k[_axisX];
			curX = (curXKey - (int)GRID_MAX_VAL) * RESOLUTION;
			mappingXKey = pixelKey.k[_axisX] - 1;
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

	void SuperRayGenerator::ComputeAxis(const point2d& _min, const point2d& _max, Axis2D& _axis) {
		// Compute traveral axis for generating a mapping lines efficiently
		// If you want the details, see "Ray Tracing Animated Scenes using Coherent Grid Traversal" by Ingo Wald et al.

		double nearDist[2];
		for (unsigned int i = 0; i < 2; i++){
			if (fabs(originW(i) - _min(i)) < fabs(originW(i) - _max(i)))	nearDist[i] = fabs(originW(i) - _min(i));
			else 														    nearDist[i] = fabs(originW(i) - _max(i));
		}

		if (nearDist[0] < nearDist[1]){
			_axis.axisU = 0;
			_axis.axisV = 1;
		}
		else{
			_axis.axisU = 1;
			_axis.axisV = 0;
		}
	}
}
