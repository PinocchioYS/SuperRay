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

#ifndef QUADMAP_CULLINGREGION_QUADTREE_H
#define QUADMAP_CULLINGREGION_QUADTREE_H

#include <quadmap/quadmap.h>
#include <quadmap_superray/SuperRayGenerator.h>

namespace quadmap{
    class CullingRegionQuadTree : public OccupancyQuadTreeBase<QuadTreeNode> {
    public:
        /// Default constructor, sets resolution of leafs
        CullingRegionQuadTree(double resolution);

        /**
         * Reads a QuadTree from a binary file
         * @param _filename
         *
         */
        CullingRegionQuadTree(std::string _filename);

        virtual ~CullingRegionQuadTree(){};

        /// virtual constructor: creates a new object of same type
        /// (Covariant return type requires an up-to-date compiler)
        CullingRegionQuadTree* create() const { return new CullingRegionQuadTree(resolution); }

        std::string getTreeType() const { return "CullingRegionQuadTree"; }

        // Super Rays and Culling Region based Updates (batch manner)

        /**
         * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
         * This function batches the leaf cells where all rays of the point clouds traverse before updates,
         * similar to computeUpdate() of quadmap::QuadTree.
         * Occupied nodes have a preference over free ones.
         *
         * @param scan Pointcloud (measurement endpoints), in global reference frame
         * @param origin measurement origin in global reference frame
         */
        virtual void insertPointCloudRays(const Pointcloud& scan, const point2d& origin);

        /**
		 * Integrate a Pointcloud (in global reference frame) using SuperRay, parallelized with OpenMP.
		 * This function converts a point clouds into superrays, and then batches the leaf cells
         * where all rays of the point clouds traverse before updates.
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
         * returning QuadTreeKeys of all nodes traversed by the beam.
         * During the traversal, the culling region stops the traversal when the ray
         * encounters the region for reducing the unnecessary traversals and updates.
         *
         * @param origin start coordinate of ray (end point of sensor ray)
         * @param end end coordinate of ray (sensor origin)
         * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding the origin cell
         * @return Success of operation. Returning false usually means that one of the coordinates is out of the QuadTree's range
         */
        bool computeInverseRayKeys(const point2d& origin, const point2d& end, KeyRay& ray, KeySet& cullingregion);


        /**
         * Static member object which ensures that this QuadTree's prototype
         * ends up in the classIDMapping only once. You need this as a
         * static member in any derived quadtree class in order to read .qt
         * files through the AbstractQuadTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer{
        public:
            StaticMemberInitializer() {
                CullingRegionQuadTree* tree = new CullingRegionQuadTree(0.1);
                tree->clearKeyRays();
                AbstractQuadTree::registerTreeType(tree);
            }

            /**
            * Dummy function to ensure that MSVC does not drop the
            * StaticMemberInitializer, causing this tree failing to register.
            * Needs to be called from the constructor of this quadtree.
            */
            void ensureLinking() {};
        };
        /// static member to ensure static initialization (only once)
        static StaticMemberInitializer cullingregionQuadTreeMemberInit;
    };
}

#endif
