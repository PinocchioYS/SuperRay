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

#include <superray_octomap/CullingRegionOcTree.h>

namespace octomap{
    CullingRegionOcTree::CullingRegionOcTree(double in_resolution)
            : OccupancyOcTreeBase<OcTreeNode>(in_resolution) {
        cullingregionOcTreeMemberInit.ensureLinking();
    }

    CullingRegionOcTree::CullingRegionOcTree(std::string _filename)
            : OccupancyOcTreeBase<OcTreeNode>(0.1)  { // resolution will be set according to tree file
        readBinary(_filename);
    }

    CullingRegionOcTree::StaticMemberInitializer CullingRegionOcTree::cullingregionOcTreeMemberInit;

    void CullingRegionOcTree::insertPointCloudRays(const Pointcloud& pc, const point3d& origin)
    {
        if (pc.size() < 1)
            return;

        // Build a culling region
        KeySet cullingregion = buildCullingRegion(pc, origin);

        // Batch a set of the updates
        KeyIntMap free_cells, occupied_cells;
#ifdef _OPENMP
        omp_set_num_threads(this->keyrays.size());
#pragma omp parallel for
#endif
        for (int i = 0; i < (int)pc.size(); ++i) {
            const point3d& p = pc[i];
            unsigned threadIdx = 0;
#ifdef _OPENMP
            threadIdx = omp_get_thread_num();
#endif
            KeyRay* keyray = &(this->keyrays.at(threadIdx));

            if (this->computeInverseRayKeys(p, origin, *keyray, cullingregion)){
#ifdef _OPENMP
#pragma omp critical (free_batch)
#endif
                {
                    // Batch the cells to be updated into the free states
                    for (KeyRay::iterator it = keyray->begin(); it != keyray->end(); ++it){
                        const KeyIntMap::iterator& cell = free_cells.find(*it);
                        if (cell == free_cells.end())
                            free_cells.insert(std::pair<OcTreeKey, int>(*it, 1));
                        else
                            cell->second = cell->second + 1;
                    }
                }

#ifdef _OPENMP
#pragma omp critical (hit_batch)
#endif
                {
                    // Batch the cells to be updated into the occupied states
                    OcTreeKey key = coordToKey(p);
                    const KeyIntMap::iterator& cell = occupied_cells.find(key);
                    if (cell == occupied_cells.end())
                        occupied_cells.insert(std::pair<OcTreeKey, int>(key, 1));
                    else
                        cell->second = cell->second + 1;
                }
            }
        }

        // Update the occupancies of the batched cells
        for (KeyIntMap::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
            updateNode(it->first, it->second * prob_miss_log);
        }
        for (KeyIntMap::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
            updateNode(it->first, it->second * prob_hit_log);
        }
    }

    void CullingRegionOcTree::insertSuperRayCloudRays(const Pointcloud& pc, const point3d& origin, const int threshold)
    {
        if (pc.size() < 1)
            return;

        // Generate the super rays
        SuperRayGenerator srgenerator(resolution, tree_max_val, threshold);
        SuperRayCloud srcloud;
        srgenerator.GenerateSuperRay(pc, origin, srcloud);

        // Build a culling region
        KeySet cullingregion = buildCullingRegion(srcloud, origin);

        // Batch a set of the updates
        KeyIntMap free_cells, occupied_cells;
#ifdef _OPENMP
        omp_set_num_threads(this->keyrays.size());
	#pragma omp parallel for
#endif
        for (int i = 0; i < (int)srcloud.size(); ++i) {
            const point3d& p = srcloud[i].p;
            unsigned threadIdx = 0;
#ifdef _OPENMP
            threadIdx = omp_get_thread_num();
#endif
            KeyRay* keyray = &(this->keyrays.at(threadIdx));

            if (this->computeInverseRayKeys(p, origin, *keyray, cullingregion)){
#ifdef _OPENMP
#pragma omp critical (free_batch)
#endif
                {
                    // Batch the cells to be updated into the free states
                    for (KeyRay::iterator it = keyray->begin(); it != keyray->end(); ++it){
                        const KeyIntMap::iterator& cell = free_cells.find(*it);
                        if (cell == free_cells.end())
                            free_cells.insert(std::pair<OcTreeKey, int>(*it, srcloud[i].w));
                        else
                            cell->second = cell->second + srcloud[i].w;
                    }
                }

#ifdef _OPENMP
#pragma omp critical (hit_batch)
#endif
                {
                    // Batch the cells to be updated into the occupied states
                    OcTreeKey key = coordToKey(p);
                    const KeyIntMap::iterator& cell = occupied_cells.find(key);
                    if (cell == occupied_cells.end())
                        occupied_cells.insert(std::pair<OcTreeKey, int>(key, srcloud[i].w));
                    else
                        cell->second = cell->second + srcloud[i].w;
                }
            }
        }

        // Update the occupancies of the batched cells
        for (KeyIntMap::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
            updateNode(it->first, it->second * prob_miss_log);
        }
        for (KeyIntMap::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
            updateNode(it->first, it->second * prob_hit_log);
        }
    }

    KeySet CullingRegionOcTree::buildCullingRegion(const point3d& origin, const int max_propagation)
    {
        KeySet cullingregion;

        OcTreeKey originKey = coordToKey(origin);

        KeySet* cur_candidates = new KeySet;
        cur_candidates->insert(originKey);

        for(int cur_level = 0; cur_level <= max_propagation; cur_level++){
            KeySet* next_candidates = new KeySet;
            for(KeySet::iterator it = cur_candidates->begin(); it != cur_candidates->end(); it++){
                // Check the insertion of the cell into the culling region
                const OcTreeKey& key = *it;

                // The first condition: does the cell have a fully free state?
                int step[3] = {0, 0, 0};
                OcTreeNode* node = search(key);
                if (!node || node->getLogOdds() > clamping_thres_min)
                    continue;

                // The second condition: are all the neighbor cells in the culling region?
                bool insertion = true;
                for (int axis = 0; axis < 3; axis++){
                    // Find a neighbor cell in the direction of the axis
                    if (key[axis] > originKey[axis])		step[axis] = -1;
                    else if (key[axis] < originKey[axis])	step[axis] = 1;

                    if (step[axis] != 0){
                        // Check a neighbor cell
                        OcTreeKey checkKey = key;
                        checkKey[axis] += step[axis];

                        // The neighbor cell is not in culling region
                        if (cullingregion.find(checkKey) == cullingregion.end()){
                            insertion = false;
                            break;
                        }
                    }
                }

                // Insert the cell into the culling region
                if(!insertion)
                    continue;
                cullingregion.insert(key);

                // Find the candidates in the next level
                if(cur_level != max_propagation){
                    for (int axis = 0; axis < 3; axis++){
                        OcTreeKey candidate = key;

                        if (step[axis] != 0){
                            candidate[axis] += (-step[axis]);
                            next_candidates->insert(candidate);
                        }
                        else{
                            candidate[axis] += 1;
                            next_candidates->insert(candidate);

                            candidate[axis] += -2;
                            next_candidates->insert(candidate);
                        }
                    }
                }
            }

            // Propagation: move to the next level
            delete cur_candidates;
            cur_candidates = next_candidates;
            if(cur_candidates->size() <= 0)
                break;
        }

        delete cur_candidates;

        return cullingregion;
    }

    KeySet CullingRegionOcTree::buildCullingRegion(const Pointcloud& pc, const point3d& origin)
    {
        // Find the maximum distance between the sensor origin and the end point
        double max_dist = 0.0;
        for(int i = 0; i < (int)pc.size(); i++){
            const point3d& p = pc[i];
            double distance = (p - origin).norm();
            if(distance > max_dist)
                max_dist = distance;
        }

        // Build a culling region limited the minimum distance
        return buildCullingRegion(origin, (int)(max_dist / resolution));
    }

    KeySet CullingRegionOcTree::buildCullingRegion(const SuperRayCloud& superrays, const point3d& origin)
    {
        // Find the maximum distance between the sensor origin and the end point
        double max_dist = 0.0;
        for(int i = 0; i < (int)superrays.size(); i++){
            const point3d& p = superrays[i].p;
            double distance = (p - origin).norm();
            if(distance > max_dist)
                max_dist = distance;
        }

        // Build a culling region limited the minimum distance
        return buildCullingRegion(origin, (int)(max_dist / resolution));
    }

    bool CullingRegionOcTree::computeInverseRayKeys(const point3d& origin, const point3d& end, KeyRay& ray, KeySet& cullingregion)
    {
        // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
        // basically: DDA in 3D

        ray.reset();

        OcTreeKey key_origin, key_end;
        if ( !coordToKeyChecked(origin, key_origin) || !coordToKeyChecked(end, key_end) ) {
            OCTOMAP_WARNING_STR("coordinates ( " << origin << " -> " << end << ") out of bounds in computeRayKeys");
            return false;
        }

        if (key_origin == key_end)
            return true; // same tree cell, we're done.

        // Initialization phase -------------------------------------------------------
        point3d direction = (end - origin);
        float length = (float) direction.norm();
        direction /= length; // normalize vector

        int    step[3];
        double tMax[3];
        double tDelta[3];

        OcTreeKey current_key = key_origin;

        for(unsigned int i = 0; i < 3; ++i) {
            // compute step direction
            if (direction(i) > 0.0)         step[i] = 1;
            else if (direction(i) < 0.0)    step[i] = -1;
            else                            step[i] = 0;

            // compute tMax, tDelta
            if (step[i] != 0) {
                // corner point of voxel (in direction of ray)
                double voxelBorder = this->keyToCoord(current_key[i]);
                voxelBorder += (float) (step[i] * this->resolution * 0.5);

                tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
                tDelta[i] = this->resolution / fabs( direction(i) );
            }
            else {
                tMax[i] =  std::numeric_limits<double>::max( );
                tDelta[i] = std::numeric_limits<double>::max( );
            }
        }

        // Incremental phase  ---------------------------------------------------------
        bool done = false;
        while (!done) {
            // find minimum tMax:
            unsigned int dim;
            if (tMax[0] < tMax[1]){
                if (tMax[0] < tMax[2]) dim = 0;
                else                   dim = 2;
            }
            else {
                if (tMax[1] < tMax[2]) dim = 1;
                else                   dim = 2;
            }

            // advance in direction "dim"
            current_key[dim] += step[dim];
            tMax[dim] += tDelta[dim];

            assert (current_key[dim] < 2*this->tree_max_val);

            // Culling out the traversal
            if (cullingregion.find(current_key) != cullingregion.end()){
                done = true;
                break;
            }
            // reached endpoint, key equv?
            else if (current_key == key_end) {
                ray.addKey(current_key);
                done = true;
                break;
            }
            else {
                // reached endpoint world coords?
                // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
                double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);
                // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
                // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
                if (dist_from_origin > length) {
                    done = true;
                    break;
                }
                else {  // continue to add freespace cells
                    ray.addKey(current_key);
                }
            }

            assert ( ray.size() < ray.sizeMax() - 1);
        } // end while

        return true;
    }
}