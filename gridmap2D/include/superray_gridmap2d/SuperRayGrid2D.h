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

#ifndef GRIDMAP2D_SUPERRAY_GRID2D_H
#define GRIDMAP2D_SUPERRAY_GRID2D_H

#include "gridmap2d.h"
#include "SuperRayGenerator.h"

namespace gridmap2d {
	class SuperRayGrid2D : public OccupancyGrid2DBase<Grid2DNode> {
	public:
		/// Default constructor, sets resolution of grid
		SuperRayGrid2D(double resolution);

		/**
		 * Reads a Grid2D from a binary file
		 * @param _filename
		 *
		 */
		// SuperRayGrid2D(std::string _filename);

		virtual ~SuperRayGrid2D(){};

		/// virtual constructor: creates a new object of same type
		/// (Covariant return type requires an up-to-date compiler)
		SuperRayGrid2D* create() const { return new SuperRayGrid2D(resolution); }

		std::string getGridType() const { return "SuperRayGrid2D"; }

		// Super Ray based Updates

		/**
		 * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
		 * This function simply inserts all rays of the point clouds, similar to insertPointCloudRays of gridmap2d::Grid2D.
		 * Occupied nodes have a preference over free ones.
		 *
		 * @param scan Pointcloud (measurement endpoints), in global reference frame
		 * @param sensor_origin measurement origin in global reference frame
		 */
		virtual void insertPointCloudRays(const Pointcloud& scan, const point2d& origin);

		/**
		 * Integrate a Pointcloud (in global reference frame) using SuperRay, parallelized with OpenMP.
		 * This function converts a point clouds into superrays, and then inserts all superrays out of the point clouds.
		 * Occupied nodes have a preference over free ones.
		 *
		 * @param scan Pointcloud (measurement endpoints), in global reference frame
		 * @param sensor_origin measurement origin in global reference frame
		 * @param threshold threshold for limiting to generate super rays
		 */
		virtual void insertSuperRayCloudRays(const Pointcloud& scan, const point2d& origin, const int threshold);

		/**
		 * Integrate a SuperRayCloud (in global reference frame), parallelized with OpenMP.
		 * This function inserts all superrays out of a point clouds.
		 * Occupied nodes have a preference over free ones.
		 *
		 * @param superray SuperRayCloud (generated by a point clouds and a origin), in global reference frame
		 */
		virtual void insertSuperRayCloudRays(const SuperRayCloud& superray);

	protected:
		/**
		 * Static member object which ensures that this Grid2D's prototype
		 * ends up in the classIDMapping only once. You need this as a
		 * static member in any derived grid2D class in order to read .og2
		 * files through the AbstractGrid2D factory. You should also call
		 * ensureLinking() once from the constructor.
		 */
		class StaticMemberInitializer{
		public:
			StaticMemberInitializer() {
				SuperRayGrid2D* grid = new SuperRayGrid2D(0.1);
				grid->clearKeyRays();
				AbstractGrid2D::registerGridType(grid);
			}

			/**
			 * Dummy function to ensure that MSVC does not drop the
			 * StaticMemberInitializer, causing this tree failing to register.
			 * Needs to be called from the constructor of this grid.
			 */
			void ensureLinking() {};
		};
		/// static member to ensure static initialization (only once)
		static StaticMemberInitializer superrayGrid2DMemberInit;
	};
}

#endif
