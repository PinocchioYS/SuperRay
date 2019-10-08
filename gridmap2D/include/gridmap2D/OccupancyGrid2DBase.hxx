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

namespace gridmap2D {

	template <class NODE>
	OccupancyGrid2DBase<NODE>::OccupancyGrid2DBase(double in_resolution)
		: Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>(in_resolution), use_bbx_limit(false), use_change_detection(false)
	{
		if (this->gridmap == NULL)
			this->gridmap = new typename Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>::OccupancyGridMap;
	}

	template <class NODE>
	OccupancyGrid2DBase<NODE>::OccupancyGrid2DBase(double in_resolution, unsigned int in_grid_max_val)
		: Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>(in_resolution, in_grid_max_val), use_bbx_limit(false), use_change_detection(false)
	{
		if (this->gridmap == NULL)
			this->gridmap = new typename Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>::OccupancyGridMap;
	}

	template <class NODE>
	OccupancyGrid2DBase<NODE>::~OccupancyGrid2DBase(){
		if (this->gridmap){
			delete this->gridmap;
			this->gridmap = NULL;
		}
	}

	template <class NODE>
	OccupancyGrid2DBase<NODE>::OccupancyGrid2DBase(const OccupancyGrid2DBase<NODE>& rhs) :
		Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>(rhs), use_bbx_limit(rhs.use_bbx_limit),
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
	void OccupancyGrid2DBase<NODE>::insertPointCloudRays(const Pointcloud& pc, const point2d& origin, double maxrange) {
		if (pc.size() < 1)
			return;

#ifdef _OPENMP
		omp_set_num_threads(this->keyrays.size());
#pragma omp parallel for
#endif
		for (int i = 0; i < (int)pc.size(); ++i) {
			const point2d& p = pc[i];
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
	NODE* OccupancyGrid2DBase<NODE>::setNodeValue(const Grid2DKey& key, float log_odds_value) {
		// clamp log odds within range:
		log_odds_value = std::min(std::max(log_odds_value, this->clamping_thres_min), this->clamping_thres_max);

		typename Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>::OccupancyGridMap::iterator cell = this->gridmap->find(key);
		if (cell == this->gridmap->end()){
			NODE* node = new NODE();
			node->setLogOdds(log_odds_value);
			this->gridmap->insert(std::pair<Grid2DKey, NODE*>(key, node));
			return node;
		}
		else{
			(cell->second)->setLogOdds(log_odds_value);
			return cell->second;
		}
	}

	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::setNodeValue(const point2d& value, float log_odds_value) {
		Grid2DKey key;
		if (!this->coordToKeyChecked(value, key))
			return NULL;

		return setNodeValue(key, log_odds_value);
	}

	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::setNodeValue(double x, double y, float log_odds_value) {
		Grid2DKey key;
		if (!this->coordToKeyChecked(x, y, key))
			return NULL;

		return setNodeValue(key, log_odds_value);
	}


	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::updateNode(const Grid2DKey& key, float log_odds_update) {
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
			this->gridmap->insert(std::pair<Grid2DKey, NODE*>(key, node));
		}
		node->addValue(log_odds_update);

		return node;
	}

	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::updateNode(const point2d& value, float log_odds_update) {
		Grid2DKey key;
		if (!this->coordToKeyChecked(value, key))
			return NULL;

		return updateNode(key, log_odds_update);
	}

	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::updateNode(double x, double y, float log_odds_update) {
		Grid2DKey key;
		if (!this->coordToKeyChecked(x, y, key))
			return NULL;

		return updateNode(key, log_odds_update);
	}

	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::updateNode(const Grid2DKey& key, bool occupied) {
		float logOdds = this->prob_miss_log;
		if (occupied)
			logOdds = this->prob_hit_log;

		return updateNode(key, logOdds);
	}

	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::updateNode(const point2d& value, bool occupied) {
		Grid2DKey key;
		if (!this->coordToKeyChecked(value, key))
			return NULL;
		return updateNode(key, occupied);
	}

	template <class NODE>
	NODE* OccupancyGrid2DBase<NODE>::updateNode(double x, double y, bool occupied) {
		Grid2DKey key;
		if (!this->coordToKeyChecked(x, y, key))
			return NULL;
		return updateNode(key, occupied);
	}

	template <class NODE>
	void OccupancyGrid2DBase<NODE>::toMaxLikelihood() {
		if (this->gridmap == NULL)
			return;

		for (typename Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>::OccupancyGridMap::iterator cell = this->gridmap->begin(); cell != this->gridmap->end(); cell++){
			nodeToMaxLikelihood(cell->second);
		}
	}

	template <class NODE>
	bool OccupancyGrid2DBase<NODE>::castRay(const point2d& origin, const point2d& directionP, point2d& end,
		bool ignoreUnknown, double maxRange) const {

		/// ----------  see Grid2DBase::computeRayKeys  -----------

		// Initialization phase -------------------------------------------------------
		Grid2DKey current_key;
		if (!Grid2DBaseImpl<NODE, AbstractOccupancyGrid2D>::coordToKeyChecked(origin, current_key)) {
			GRIDMAP2D_WARNING_STR("Coordinates out of bounds during ray casting");
			return false;
		}

		NODE* startingNode = this->search(current_key);
		if (startingNode){
			if (this->isNodeOccupied(startingNode)){
				// Occupied node found at origin 
				// (need to convert from key, since origin does not need to be a pixel center)
				end = this->keyToCoord(current_key);
				return true;
			}
		}
		else if (!ignoreUnknown){
			end = this->keyToCoord(current_key);
			return false;
		}

		point2d direction = directionP.normalized();
		bool max_range_set = (maxRange > 0.0);

		int step[2];
		double tMax[2];
		double tDelta[2];

		for (unsigned int i = 0; i < 2; ++i) {
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

		if (step[0] == 0 && step[1] == 0){
			GRIDMAP2D_ERROR("Raycasting in direction (0,0) is not possible!");
			return false;
		}

		// for speedup:
		double maxrange_sq = maxRange * maxRange;

		// Incremental phase  ---------------------------------------------------------

		bool done = false;

		while (!done) {
			unsigned int dim;

			// find minimum tMax:
			dim = tMax[0] < tMax[1] ? 0 : 1;

			// check for overflow:
			if ((step[dim] < 0 && current_key[dim] == 0)
				|| (step[dim] > 0 && current_key[dim] == 2 * this->grid_max_val - 1))
			{
				GRIDMAP2D_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
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
				for (unsigned int j = 0; j < 2; j++) {
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
	bool OccupancyGrid2DBase<NODE>::getRayIntersection(const point2d& origin, const point2d& direction, const point2d& center,
		point2d& intersection, double delta) const {
		// We only need three normals for the six planes
		gridmap2D::point2d normalX(1, 0);
		gridmap2D::point2d normalY(0, 1);

		// One point on each plane, let them be the center for simplicity
		gridmap2D::point2d pointXNeg(center(0) - float(this->resolution / 2.0), center(1));
		gridmap2D::point2d pointXPos(center(0) + float(this->resolution / 2.0), center(1));
		gridmap2D::point2d pointYNeg(center(0), center(1) - float(this->resolution / 2.0));
		gridmap2D::point2d pointYPos(center(0), center(1) + float(this->resolution / 2.0));

		double lineDotNormal = 0.0;
		double d = 0.0;
		double outD = std::numeric_limits<double>::max();
		gridmap2D::point2d intersect;
		bool found = false;

		// Find the intersection (if any) with each place
		// Line dot normal will be zero if they are parallel, in which case no intersection can be the entry one
		// if there is an intersection does it occur in the bounded plane of the voxel
		// if yes keep only the closest (smallest distance to sensor origin).
		if ((lineDotNormal = normalX.dot(direction))){
			d = (pointXNeg - origin).dot(normalX) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}

			d = (pointXPos - origin).dot(normalX) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}
		}

		if ((lineDotNormal = normalY.dot(direction))){
			d = (pointYNeg - origin).dot(normalY) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}

			d = (pointYPos - origin).dot(normalY) / lineDotNormal;
			intersect = direction * float(d) + origin;
			if (!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6))){
				outD = std::min(outD, d);
				found = true;
			}
		}

		// Substract (add) a fraction to ensure no ambiguity on the starting pixel
		// Don't start on a bondary.
		if (found)
			intersection = direction * float(outD + delta) + origin;

		return found;
	}

	template <class NODE> inline bool
		OccupancyGrid2DBase<NODE>::integrateMissOnRay(const point2d& origin, const point2d& end) {

		if (!this->computeRayKeys(origin, end, this->keyrays.at(0))) {
			return false;
		}

		for (KeyRay::iterator it = this->keyrays[0].begin(); it != this->keyrays[0].end(); it++) {
			updateNode(*it, false); // insert freespace measurement
		}

		return true;
	}

	template <class NODE> bool
		OccupancyGrid2DBase<NODE>::insertRay(const point2d& origin, const point2d& end, double maxrange)
	{
		// cut ray at maxrange
		if ((maxrange > 0) && ((end - origin).norm() > maxrange))
		{
			point2d direction = (end - origin).normalized();
			point2d new_end = origin + direction * (float)maxrange;
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
	void OccupancyGrid2DBase<NODE>::setBBXMin(point2d& min) {
		bbx_min = min;
		if (!this->coordToKeyChecked(bbx_min, bbx_min_key)) {
			GRIDMAP2D_ERROR("ERROR while generating bbx min key.\n");
		}
	}

	template <class NODE>
	void OccupancyGrid2DBase<NODE>::setBBXMax(point2d& max) {
		bbx_max = max;
		if (!this->coordToKeyChecked(bbx_max, bbx_max_key)) {
			GRIDMAP2D_ERROR("ERROR while generating bbx max key.\n");
		}
	}

	template <class NODE>
	bool OccupancyGrid2DBase<NODE>::inBBX(const point2d& p) const {
		return ((p.x() >= bbx_min.x()) && (p.y() >= bbx_min.y()) && (p.x() <= bbx_max.x()) && (p.y() <= bbx_max.y()));
	}

	template <class NODE>
	bool OccupancyGrid2DBase<NODE>::inBBX(const Grid2DKey& key) const {
		return ((key[0] >= bbx_min_key[0]) && (key[1] >= bbx_min_key[1]) &&	(key[0] <= bbx_max_key[0]) && (key[1] <= bbx_max_key[1]));
	}

	template <class NODE>
	point2d OccupancyGrid2DBase<NODE>::getBBXBounds() const {
		gridmap2D::point2d obj_bounds = (bbx_max - bbx_min);
		obj_bounds /= 2.;
		return obj_bounds;
	}

	template <class NODE>
	point2d OccupancyGrid2DBase<NODE>::getBBXCenter() const {
		gridmap2D::point2d obj_bounds = (bbx_max - bbx_min);
		obj_bounds /= 2.;
		return bbx_min + obj_bounds;
	}


	//-- Occupancy queries on nodes:

	template <class NODE>
	void OccupancyGrid2DBase<NODE>::updateNodeLogOdds(NODE* occupancyNode, const float& update) const {
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
	void OccupancyGrid2DBase<NODE>::integrateHit(NODE* occupancyNode) const {
		updateNodeLogOdds(occupancyNode, this->prob_hit_log);
	}

	template <class NODE>
	void OccupancyGrid2DBase<NODE>::integrateMiss(NODE* occupancyNode) const {
		updateNodeLogOdds(occupancyNode, this->prob_miss_log);
	}

	template <class NODE>
	void OccupancyGrid2DBase<NODE>::nodeToMaxLikelihood(NODE* occupancyNode) const{
		if (this->isNodeOccupied(occupancyNode))
			occupancyNode->setLogOdds(this->clamping_thres_max);
		else
			occupancyNode->setLogOdds(this->clamping_thres_min);
	}

	template <class NODE>
	void OccupancyGrid2DBase<NODE>::nodeToMaxLikelihood(NODE& occupancyNode) const{
		if (this->isNodeOccupied(occupancyNode))
			occupancyNode.setLogOdds(this->clamping_thres_max);
		else
			occupancyNode.setLogOdds(this->clamping_thres_min);
	}

    template <class NODE>
    std::istream& OccupancyGrid2DBase<NODE>::readBinaryData(std::istream &s){
        if (this->size() > 0) {
            GRIDMAP2D_ERROR_STR("Trying to read into an existing grid.");
            return s;
        }

        size_t number_of_cells = 0;
        s.read((char*)&number_of_cells, sizeof(number_of_cells));

        std::vector<Grid2DKey> key_list(8);
        std::bitset<8> binary_occupancy;	// 1: occupied, 0: free
        for(size_t i = 0; i < number_of_cells; i++){
            s.read((char*)key_list[i%8].k, sizeof(key_list[i%8].k));

            if(i % 8 == 7){
                char binary_occupancy_char = 0;
                s.read((char*)&binary_occupancy_char, sizeof(char));
                std::bitset<8> binary_occupancy((unsigned long long) binary_occupancy_char);	// 1: occupied, 0: free

                for(unsigned int j = 0; j < 8; j++){
                    Grid2DNode* new_node = new Grid2DNode();
                    if(binary_occupancy[j] == 1)
                        new_node->setLogOdds(this->clamping_thres_max);
                    else
                        new_node->setLogOdds(this->clamping_thres_min);
                    this->gridmap->insert(std::pair<Grid2DKey, Grid2DNode*>(key_list[j], new_node));
                }
            }
        }

        if(number_of_cells % 8 != 0){
            char binary_occupancy_char = 0;
            s.read((char*)&binary_occupancy_char, sizeof(char));
            std::bitset<8> binary_occupancy((unsigned long long) binary_occupancy_char);	// 1: occupied, 0: free

            for(unsigned int j = 0; j <= (number_of_cells % 8); j++){
                Grid2DNode* new_node = new Grid2DNode();
                if(binary_occupancy[j] == 1)
                    new_node->setLogOdds(this->clamping_thres_max);
                else
                    new_node->setLogOdds(this->clamping_thres_min);
                this->gridmap->insert(std::pair<Grid2DKey, Grid2DNode*>(key_list[j], new_node));
            }
        }

        this->size_changed = true;

        return s;
    }

	template <class NODE>
	std::ostream& OccupancyGrid2DBase<NODE>::writeBinaryData(std::ostream &s) const{
		GRIDMAP2D_DEBUG("Writing %zu nodes to output stream...", this->size());

		size_t number_of_cells = this->size();
		s.write((char*)&number_of_cells, sizeof(number_of_cells));

		typename OccupancyGrid2DBase<NODE>::OccupancyGridMap::iterator it = this->gridmap->begin();
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
