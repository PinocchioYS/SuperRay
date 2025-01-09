/*
* Copyright(c) 2019, Youngsun Kwon, Donghyuk Kim, Inkyu An, and Sung-eui Yoon, KAIST
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

#ifndef GRIDMAP2D_CULLINGREGION_GRID2D_H
#define GRIDMAP2D_CULLINGREGION_GRID2D_H

#include "gridmap2d.h"
#include "SuperRayGenerator.h"

namespace gridmap2d {
    class CullingRegionGrid2D : public OccupancyGrid2DBase<Grid2DNode> {
    public:
        /// Default constructor, sets resolution of grid
        CullingRegionGrid2D(double resolution);

        /**
         * Reads a Grid2D from a binary file
         * @param _filename
         *
         */
        // CullingRegionGrid2D(std::string _filename);

        virtual ~CullingRegionGrid2D(){};

        /// virtual constructor: creates a new object of same type
        /// (Covariant return type requires an up-to-date compiler)
        CullingRegionGrid2D* create() const { return new CullingRegionGrid2D(resolution); }

        std::string getGridType() const { return "CullingRegionGrid2D"; }

        // Super Rays and Culling Region based Updates

        /**
		 * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
		 * This function simply inserts all rays of the point clouds, similar to insertPointCloudRays of gridmap2d::Grid2D.
		 * Occupied nodes have a preference over free ones.
		 *
		 * @param scan Pointcloud (measurement endpoints), in global reference frame
		 * @param origin measurement origin in global reference frame
		 */
        virtual void insertPointCloudRays(const Pointcloud& scan, const point2d& origin);

        /**
		 * Integrate a Pointcloud (in global reference frame) using SuperRay, parallelized with OpenMP.
		 * This function converts a point clouds into superrays, and then inserts all superrays out of the point clouds.
		 * Occupied nodes have a preference over free ones.
		 *
		 * @param scan Pointcloud (measurement endpoints), in global reference frame
		 * @param origin measurement origin in global reference frame
		 * @param threshold threshold for limiting to generate super rays
		 */
        virtual void insertSuperRayCloudRays(const Pointcloud& scan, const point2d& origin, const int threshold);

    protected:
        /**
		 * Build a culling region by utilizing the occupancy information updated to the map.
		 * The implementation is based on a priority queue according to the Manhattan distance from the origin cell.
         *
		 * @param origin measurement origin in global reference frame
		 * @param max_propagation maximum level of propagation; Manhattan distance from the origin cell
         * @return culling region limited by the maximum level of the propagation
		 */
        KeySet buildCullingRegion(const point2d& origin, const int max_propagation);

        /**
		 * Build a culling region by utilizing the occupancy information updated to the map.
         *
         * @param scan Pointcloud (measurement endpoints), in global reference frame
		 * @param origin measurement origin in global reference frame
         * @return culling region limited by the range of measurements
		 */
        KeySet buildCullingRegion(const Pointcloud& scan, const point2d& origin);

        /**
		 * Build a culling region by utilizing the occupancy information updated to the map.
         *
         * @param superrays Super rays computed from the measurements in global reference frame
		 * @param origin measurement origin in global reference frame
         * @return culling region limited by the range of measurements
		 */
        KeySet buildCullingRegion(const SuperRayCloud& superrays, const point2d& origin);

        /**
         * Traces a sensor ray from origin (excluding) to end in the inverse direction (see the description),
         * returning Grid2DKeys of all nodes traversed by the beam.
         * During the traversal, the culling region stops the traversal when the ray
         * encounters the region for reducing the unnecessary traversals and updates.
         *
         * @param origin start coordinate of ray (end point of sensor ray)
         * @param end end coordinate of ray (sensor origin)
         * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding the origin cell
         * @return Success of operation. Returning false usually means that one of the coordinates is out of the Grid2D's range
         */
        bool computeInverseRayKeys(const point2d& origin, const point2d& end, KeyRay& ray, KeySet& cullingregion);


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
                CullingRegionGrid2D* grid = new CullingRegionGrid2D(0.1);
                grid->clearKeyRays();
                AbstractGrid2D::registerGridType(grid);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the
             * StaticMemberInitializer, causing this grid failing to register.
             * Needs to be called from the constructor of this grid2D.
             */
            void ensureLinking() {};
        };
        /// static member to ensure static initialization (only once)
        static StaticMemberInitializer cullingregionGrid2DMemberInit;
    };
}

#endif
