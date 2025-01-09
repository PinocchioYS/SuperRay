/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <bitset>
#include <algorithm>

namespace gridmap3D {

	template <class NODE>
	OccupancyGrid3DBase<NODE>::OccupancyGrid3DBase(double in_resolution)
		: Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>(in_resolution), use_bbx_limit(false), use_change_detection(false)
	{
		if (this->gridmap == NULL)
			this->gridmap = new typename Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>::OccupancyGridMap;
	}

	template <class NODE>
	OccupancyGrid3DBase<NODE>::OccupancyGrid3DBase(double in_resolution, unsigned int in_grid_max_val)
		: Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>(in_resolution, in_grid_max_val), use_bbx_limit(false), use_change_detection(false)
	{
		if (this->gridmap == NULL)
			this->gridmap = new typename Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>::OccupancyGridMap;
	}

	template <class NODE>
	OccupancyGrid3DBase<NODE>::~OccupancyGrid3DBase(){
		if (this->gridmap){
			delete this->gridmap;
			this->gridmap = NULL;
		}
	}

	template <class NODE>
	OccupancyGrid3DBase<NODE>::OccupancyGrid3DBase(const OccupancyGrid3DBase<NODE>& rhs) :
		Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>(rhs), use_bbx_limit(rhs.use_bbx_limit),
		bbx_min(rhs.bbx_min), bbx_max(rhs.bbx_max),
		bbx_min_key(rhs.bbx_min_key), bbx_max_key(rhs.bbx_max_key),
		use_change_detection(rhs.use_change_detection), changed_keys(rhs.changed_keys)
	{
		this->clamping_thres_min = rhs.clamping_thres_min;
		this->clamping_thres_max = rhs.clamping_thres_max;
		this->prob_hit_log = rhs.prob_hit_log;
		this->prob_miss_log = rhs.prob_miss_log;
		this->occ_prob_thres_log = rhs.occ_prob_thres_log;
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::insertPointCloudRays(const Pointcloud& pc, const point3d& origin, double maxrange) {
		if (pc.size() < 1)
			return;

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

			if (this->computeRayKeys(origin, p, *keyray)){
#ifdef _OPENMP
#pragma omp critical
#endif
				{
					for (KeyRay::iterator it = keyray->begin(); it != keyray->end(); it++) {
						updateNode(*it, false); // insert freespace measurement
					}
					updateNode(p, true); // update endpoint to be occupied
				}
			}

		}
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::setNodeValue(const Grid3DKey& key, float log_odds_value) {
		// clamp log odds within range:
		log_odds_value = std::min(std::max(log_odds_value, this->clamping_thres_min), this->clamping_thres_max);

		typename Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>::OccupancyGridMap::iterator cell = this->gridmap->find(key);
		if (cell == this->gridmap->end()){
			NODE* node = new NODE();
			node->setLogOdds(log_odds_value);
			this->gridmap->insert(std::pair<Grid3DKey, NODE*>(key, node));
			return node;
		}
		else{
			(cell->second)->setLogOdds(log_odds_value);
			return cell->second;
		}
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::setNodeValue(const point3d& value, float log_odds_value) {
		Grid3DKey key;
		if (!this->coordToKeyChecked(value, key))
			return NULL;

		return setNodeValue(key, log_odds_value);
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::setNodeValue(double x, double y, double z, float log_odds_value) {
		Grid3DKey key;
		if (!this->coordToKeyChecked(x, y, z, key))
			return NULL;

		return setNodeValue(key, log_odds_value);
	}


	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::updateNode(const Grid3DKey& key, float log_odds_update) {
		// early abort (no change will happen).
		// may cause an overhead in some configuration, but more often helps
		NODE* node = this->search(key);
		// no change: node already at threshold
		if (node
			&& ((log_odds_update >= 0 && node->getLogOdds() >= this->clamping_thres_max)
			|| (log_odds_update <= 0 && node->getLogOdds() <= this->clamping_thres_min)))
		{
			return node;
		}

		if (!node){
			node = new NODE();
			this->gridmap->insert(std::pair<Grid3DKey, NODE*>(key, node));
		}
		node->addValue(log_odds_update);

		return node;
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::updateNode(const point3d& value, float log_odds_update) {
		Grid3DKey key;
		if (!this->coordToKeyChecked(value, key))
			return NULL;

		return updateNode(key, log_odds_update);
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::updateNode(double x, double y, double z, float log_odds_update) {
		Grid3DKey key;
		if (!this->coordToKeyChecked(x, y, z, key))
			return NULL;

		return updateNode(key, log_odds_update);
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::updateNode(const Grid3DKey& key, bool occupied) {
		float logOdds = this->prob_miss_log;
		if (occupied)
			logOdds = this->prob_hit_log;

		return updateNode(key, logOdds);
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::updateNode(const point3d& value, bool occupied) {
		Grid3DKey key;
		if (!this->coordToKeyChecked(value, key))
			return NULL;
		return updateNode(key, occupied);
	}

	template <class NODE>
	NODE* OccupancyGrid3DBase<NODE>::updateNode(double x, double y, double z, bool occupied) {
		Grid3DKey key;
		if (!this->coordToKeyChecked(x, y, z, key))
			return NULL;
		return updateNode(key, occupied);
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::toMaxLikelihood() {
		if (this->gridmap == NULL)
			return;

		for (typename Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>::OccupancyGridMap::iterator cell = this->gridmap->begin(); cell != this->gridmap->end(); cell++){
			nodeToMaxLikelihood(cell->second);
		}
	}

/*	template <class NODE>
	bool OccupancyQuadTreeBase<NODE>::getNormals(const point3d& point, std::vector<point3d>& normals,
		bool unknownStatus) const {
		normals.clear();

		OcTreeKey init_key;
		if (!OcTreeBaseImpl<NODE, AbstractOccupancyOcTree>::coordToKeyChecked(point, init_key)) {
			OCTOMAP_WARNING_STR("Voxel out of bounds");
			return false;
		}

		// OCTOMAP_WARNING("Normal for %f, %f, %f\n", point.x(), point.y(), point.z());

		int vertex_values[8];

		OcTreeKey current_key;
		NODE* current_node;

		// There is 8 neighbouring sets
		// The current cube can be at any of the 8 vertex
		int x_index[4][4] = { { 1, 1, 0, 0 }, { 1, 1, 0, 0 }, { 0, 0 - 1, -1 }, { 0, 0 - 1, -1 } };
		int y_index[4][4] = { { 1, 0, 0, 1 }, { 0, -1, -1, 0 }, { 0, -1, -1, 0 }, { 1, 0, 0, 1 } };
		int z_index[2][2] = { { 0, 1 }, { -1, 0 } };

		// Iterate over the 8 neighboring sets
		for (int m = 0; m < 2; ++m){
			for (int l = 0; l < 4; ++l){

				int k = 0;
				// Iterate over the cubes
				for (int j = 0; j < 2; ++j){
					for (int i = 0; i < 4; ++i){
						current_key[0] = init_key[0] + x_index[l][i];
						current_key[1] = init_key[1] + y_index[l][i];
						current_key[2] = init_key[2] + z_index[m][j];
						current_node = this->search(current_key);

						if (current_node){
							vertex_values[k] = this->isNodeOccupied(current_node);

							// point3d coord = this->keyToCoord(current_key);
							// OCTOMAP_WARNING_STR("vertex " << k << " at " << coord << "; value " << vertex_values[k]);
						}
						else{
							// Occupancy of unknown cells
							vertex_values[k] = unknownStatus;
						}
						++k;
					}
				}

				int cube_index = 0;
				if (vertex_values[0]) cube_index |= 1;
				if (vertex_values[1]) cube_index |= 2;
				if (vertex_values[2]) cube_index |= 4;
				if (vertex_values[3]) cube_index |= 8;
				if (vertex_values[4]) cube_index |= 16;
				if (vertex_values[5]) cube_index |= 32;
				if (vertex_values[6]) cube_index |= 64;
				if (vertex_values[7]) cube_index |= 128;

				// OCTOMAP_WARNING_STR("cubde_index: " << cube_index);

				// All vertices are occupied or free resulting in no normal
				if (edgeTable[cube_index] == 0)
					return true;

				// No interpolation is done yet, we use vertexList in <MCTables.h>.
				for (int i = 0; triTable[cube_index][i] != -1; i += 3){
					point3d p1 = vertexList[triTable[cube_index][i]];
					point3d p2 = vertexList[triTable[cube_index][i + 1]];
					point3d p3 = vertexList[triTable[cube_index][i + 2]];
					point3d v1 = p2 - p1;
					point3d v2 = p3 - p1;

					// OCTOMAP_WARNING("Vertex p1 %f, %f, %f\n", p1.x(), p1.y(), p1.z());
					// OCTOMAP_WARNING("Vertex p2 %f, %f, %f\n", p2.x(), p2.y(), p2.z());
					// OCTOMAP_WARNING("Vertex p3 %f, %f, %f\n", p3.x(), p3.y(), p3.z());

					// Right hand side cross product to retrieve the normal in the good
					// direction (pointing to the free nodes).
					normals.push_back(v1.cross(v2).normalize());
				}
			}
		}

		return true;
	}*/

	template <class NODE>
	bool OccupancyGrid3DBase<NODE>::castRay(const point3d& origin, const point3d& directionP, point3d& end,
		bool ignoreUnknown, double maxRange) const {

		/// ----------  see Grid3DBase::computeRayKeys  -----------

		// Initialization phase -------------------------------------------------------
		Grid3DKey current_key;
		if (!Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D>::coordToKeyChecked(origin, current_key)) {
			GRIDMAP3D_WARNING_STR("Coordinates out of bounds during ray casting");
			return false;
		}

		NODE* startingNode = this->search(current_key);
		if (startingNode){
			if (this->isNodeOccupied(startingNode)){
				// Occupied node found at origin 
				// (need to convert from key, since origin does not need to be a voxel center)
				end = this->keyToCoord(current_key);
				return true;
			}
		}
		else if (!ignoreUnknown){
			end = this->keyToCoord(current_key);
			return false;
		}

		point3d direction = directionP.normalized();
		bool max_range_set = (maxRange > 0.0);

		int step[3];
		double tMax[3];
		double tDelta[3];

		for (unsigned int i = 0; i < 3; ++i) {
			// compute step direction
			if (direction(i) > 0.0) step[i] = 1;
			else if (direction(i) < 0.0)   step[i] = -1;
			else step[i] = 0;

			// compute tMax, tDelta
			if (step[i] != 0) {
				// corner point of voxel (in direction of ray)
				double voxelBorder = this->keyToCoord(current_key[i]);
				voxelBorder += double(step[i] * this->resolution * 0.5);

				tMax[i] = (voxelBorder - origin(i)) / direction(i);
				tDelta[i] = this->resolution / fabs(direction(i));
			}
			else {
				tMax[i] = std::numeric_limits<double>::max();
				tDelta[i] = std::numeric_limits<double>::max();
			}
		}

		if (step[0] == 0 && step[1] == 0 && step[2] == 0){
			GRIDMAP3D_ERROR("Raycasting in direction (0,0,0) is not possible!");
			return false;
		}

		// for speedup:
		double maxrange_sq = maxRange * maxRange;

		// Incremental phase  ---------------------------------------------------------

		bool done = false;

		while (!done) {
			unsigned int dim;

			// find minimum tMax:
			if (tMax[0] < tMax[1]){
				if (tMax[0] < tMax[2]) dim = 0;
				else                   dim = 2;
			}
			else {
				if (tMax[1] < tMax[2]) dim = 1;
				else                   dim = 2;
			}

			// check for overflow:
			if ((step[dim] < 0 && current_key[dim] == 0)
				|| (step[dim] > 0 && current_key[dim] == 2 * this->grid_max_val - 1))
			{
				GRIDMAP3D_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
				// return border point nevertheless:
				end = this->keyToCoord(current_key);
				return false;
			}

			// advance in direction "dim"
			current_key[dim] += step[dim];
			tMax[dim] += tDelta[dim];


			// generate world coords from key
			end = this->keyToCoord(current_key);

			// check for maxrange:
			if (max_range_set){
				double dist_from_origin_sq(0.0);
				for (unsigned int j = 0; j < 3; j++) {
					dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
				}
				if (dist_from_origin_sq > maxrange_sq)
					return false;

			}

			NODE* currentNode = this->search(current_key);
			if (currentNode){
				if (this->isNodeOccupied(currentNode)) {
					done = true;
					break;
				}
				// otherwise: node is free and valid, raycasting continues
			}
			else if (!ignoreUnknown){ // no node found, this usually means we are in "unknown" areas
				return false;
			}
		} // end while

		return true;
	}

	template <class NODE>
	bool OccupancyGrid3DBase<NODE>::getRayIntersection(const point3d& origin, const point3d& direction, const point3d& center,
													   point3d& intersection, double delta) const {
		// We only need three normals for the six planes
		gridmap3D::point3d normalX(1, 0, 0);
		gridmap3D::point3d normalY(0, 1, 0);
		gridmap3D::point3d normalZ(0, 0, 1);

		// One point on each plane, let them be the center for simplicity
		gridmap3D::point3d pointXNeg(center(0) - float(this->resolution / 2.0), center(1), center(2));
		gridmap3D::point3d pointXPos(center(0) + float(this->resolution / 2.0), center(1), center(2));
		gridmap3D::point3d pointYNeg(center(0), center(1) - float(this->resolution / 2.0), center(2));
		gridmap3D::point3d pointYPos(center(0), center(1) + float(this->resolution / 2.0), center(2));
		gridmap3D::point3d pointZNeg(center(0), center(1), center(2) - float(this->resolution / 2.0));
		gridmap3D::point3d pointZPos(center(0), center(1), center(2) + float(this->resolution / 2.0));

		double lineDotNormal = 0.0;
		double d = 0.0;
		double outD = std::numeric_limits<double>::max();
		gridmap3D::point3d intersect;
		bool found = false;

		// Find the intersection (if any) with each place
		// Line dot normal will be zero if they are parallel, in which case no intersection can be the entry one
		// if there is an intersection does it occur in the bounded plane of the voxel
		// if yes keep only the closest (smallest distance to sensor origin).
		if ((lineDotNormal = normalX.dot(direction)) != 0.0){   // Ensure lineDotNormal is non-zero (assign and test)
			d = (pointXNeg - origin).dot(normalX) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6) ||
				intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}

			d = (pointXPos - origin).dot(normalX) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6) ||
				intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}
		}

		if ((lineDotNormal = normalY.dot(direction)) != 0.0){   // Ensure lineDotNormal is non-zero (assign and test)
			d = (pointYNeg - origin).dot(normalY) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
				intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}

			d = (pointYPos - origin).dot(normalY) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
				intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}
		}

		if ((lineDotNormal = normalZ.dot(direction)) != 0.0){   // Ensure lineDotNormal is non-zero (assign and test)
			d = (pointZNeg - origin).dot(normalZ) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
				intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}

			d = (pointZPos - origin).dot(normalZ) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
				intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}
		}

		// Substract (add) a fraction to ensure no ambiguity on the starting voxel
		// Don't start on a boundary.
		if (found)
			intersection = direction * float(outD + delta) + origin;

		return found;
	}

	template <class NODE> inline bool
		OccupancyGrid3DBase<NODE>::integrateMissOnRay(const point3d& origin, const point3d& end) {

		if (!this->computeRayKeys(origin, end, this->keyrays.at(0))) {
			return false;
		}

		for (KeyRay::iterator it = this->keyrays[0].begin(); it != this->keyrays[0].end(); it++) {
			updateNode(*it, false); // insert freespace measurement
		}

		return true;
	}

	template <class NODE> bool
		OccupancyGrid3DBase<NODE>::insertRay(const point3d& origin, const point3d& end, double maxrange)
	{
		// cut ray at maxrange
		if ((maxrange > 0) && ((end - origin).norm() > maxrange))
		{
			point3d direction = (end - origin).normalized();
			point3d new_end = origin + direction * (float)maxrange;
			return integrateMissOnRay(origin, new_end);
		}
		// insert complete ray
		else
		{
			if (!integrateMissOnRay(origin, end))
				return false;
			updateNode(end, true); // insert hit cell
			return true;
		}
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::setBBXMin(const point3d& min) {
		bbx_min = min;
		if (!this->coordToKeyChecked(bbx_min, bbx_min_key)) {
			GRIDMAP3D_ERROR("ERROR while generating bbx min key.\n");
		}
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::setBBXMax(const point3d& max) {
		bbx_max = max;
		if (!this->coordToKeyChecked(bbx_max, bbx_max_key)) {
			GRIDMAP3D_ERROR("ERROR while generating bbx max key.\n");
		}
	}

	template <class NODE>
	bool OccupancyGrid3DBase<NODE>::inBBX(const point3d& p) const {
		return ((p.x() >= bbx_min.x()) && (p.y() >= bbx_min.y()) && (p.z() >= bbx_min.z()) &&
			(p.x() <= bbx_max.x()) && (p.y() <= bbx_max.y()) && (p.z() <= bbx_max.z()));
	}

	template <class NODE>
	bool OccupancyGrid3DBase<NODE>::inBBX(const Grid3DKey& key) const {
		return ((key[0] >= bbx_min_key[0]) && (key[1] >= bbx_min_key[1]) && (key[2] >= bbx_min_key[2]) &&
			(key[0] <= bbx_max_key[0]) && (key[1] <= bbx_max_key[1]) && (key[2] <= bbx_max_key[2]));
	}

	template <class NODE>
	point3d OccupancyGrid3DBase<NODE>::getBBXBounds() const {
		gridmap3D::point3d obj_bounds = (bbx_max - bbx_min);
		obj_bounds /= 2.;
		return obj_bounds;
	}

	template <class NODE>
	point3d OccupancyGrid3DBase<NODE>::getBBXCenter() const {
		gridmap3D::point3d obj_bounds = (bbx_max - bbx_min);
		obj_bounds /= 2.;
		return bbx_min + obj_bounds;
	}

	
	//-- Occupancy queries on nodes:

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::updateNodeLogOdds(NODE* occupancyNode, const float& update) const {
		occupancyNode->addValue(update);
		if (occupancyNode->getLogOdds() < this->clamping_thres_min) {
			occupancyNode->setLogOdds(this->clamping_thres_min);
			return;
		}
		if (occupancyNode->getLogOdds() > this->clamping_thres_max) {
			occupancyNode->setLogOdds(this->clamping_thres_max);
		}
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::integrateHit(NODE* occupancyNode) const {
		updateNodeLogOdds(occupancyNode, this->prob_hit_log);
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::integrateMiss(NODE* occupancyNode) const {
		updateNodeLogOdds(occupancyNode, this->prob_miss_log);
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::nodeToMaxLikelihood(NODE* occupancyNode) const{
		if (this->isNodeOccupied(occupancyNode))
			occupancyNode->setLogOdds(this->clamping_thres_max);
		else
			occupancyNode->setLogOdds(this->clamping_thres_min);
	}

	template <class NODE>
	void OccupancyGrid3DBase<NODE>::nodeToMaxLikelihood(NODE& occupancyNode) const{
		if (this->isNodeOccupied(occupancyNode))
			occupancyNode.setLogOdds(this->clamping_thres_max);
		else
			occupancyNode.setLogOdds(this->clamping_thres_min);
	}

	template <class NODE>
	std::istream& OccupancyGrid3DBase<NODE>::readBinaryData(std::istream &s){
		if (this->size() > 0) {
			GRIDMAP3D_ERROR_STR("Trying to read into an existing grid.");
			return s;
		}

		size_t number_of_cells = 0;
		s.read((char*)&number_of_cells, sizeof(number_of_cells));

		std::vector<Grid3DKey> key_list(8);
		std::bitset<8> binary_occupancy;	// 1: occupied, 0: free
		for(size_t i = 0; i < number_of_cells; i++){
			s.read((char*)key_list[i%8].k, sizeof(key_list[i%8].k));

			if(i % 8 == 7){
				char binary_occupancy_char = 0;
				s.read((char*)&binary_occupancy_char, sizeof(char));
				std::bitset<8> binary_occupancy((unsigned long long) binary_occupancy_char);	// 1: occupied, 0: free

				for(unsigned int j = 0; j < 8; j++){
					Grid3DNode* new_node = new Grid3DNode();
					if(binary_occupancy[j] == 1)
						new_node->setLogOdds(this->clamping_thres_max);
					else
						new_node->setLogOdds(this->clamping_thres_min);
					this->gridmap->insert(std::pair<Grid3DKey, Grid3DNode*>(key_list[j], new_node));
				}
			}
		}

		if(number_of_cells % 8 != 0){
			char binary_occupancy_char = 0;
			s.read((char*)&binary_occupancy_char, sizeof(char));
			std::bitset<8> binary_occupancy((unsigned long long) binary_occupancy_char);	// 1: occupied, 0: free

			for(unsigned int j = 0; j <= (number_of_cells % 8); j++){
				Grid3DNode* new_node = new Grid3DNode();
				if(binary_occupancy[j] == 1)
					new_node->setLogOdds(this->clamping_thres_max);
				else
					new_node->setLogOdds(this->clamping_thres_min);
				this->gridmap->insert(std::pair<Grid3DKey, Grid3DNode*>(key_list[j], new_node));
			}
		}

		this->size_changed = true;

		return s;
	}

	template <class NODE>
	std::ostream& OccupancyGrid3DBase<NODE>::writeBinaryData(std::ostream &s) const{
		GRIDMAP3D_DEBUG("Writing %zu nodes to output stream...", this->size());

		size_t number_of_cells = this->size();
		s.write((char*)&number_of_cells, sizeof(number_of_cells));

		typename OccupancyGrid3DBase<NODE>::OccupancyGridMap::iterator it = this->gridmap->begin();
		std::bitset<8> binary_occupancy;	// 1: occupied, 0: free
		for(size_t i = 0; i < number_of_cells; i++, it++){
			s.write((char*) it->first.k, sizeof(it->first.k));
			binary_occupancy[i % 8] = this->isNodeOccupied(it->second) ? 1 : 0;

			if(i % 8 == 7){
				char binary_occupancy_char = (char)binary_occupancy.to_ulong();
				s.write((char*)&binary_occupancy_char, sizeof(char));
				binary_occupancy.reset();
			}
		}

		if(this->size() % 8 != 0) {
			char binary_occupancy_char = (char)binary_occupancy.to_ulong();
			s.write((char*)&binary_occupancy_char, sizeof(char));
		}

		return s;
	}

} // namespace
