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

#undef max
#undef min
#include <limits>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace gridmap2d {

	template <class NODE, class I>
	Grid2DBaseImpl<NODE, I>::Grid2DBaseImpl(double in_resolution) :
		I(), gridmap(NULL), grid_max_val(32768),
		resolution(in_resolution)
	{
		init();
	}

	template <class NODE, class I>
	Grid2DBaseImpl<NODE, I>::Grid2DBaseImpl(double in_resolution, unsigned int in_grid_max_val) :
		I(), gridmap(NULL), grid_max_val(in_grid_max_val),
		resolution(in_resolution)
	{
		init();
	}


	template <class NODE, class I>
	Grid2DBaseImpl<NODE, I>::~Grid2DBaseImpl(){
		clear();
	}


	template <class NODE, class I>
	Grid2DBaseImpl<NODE, I>::Grid2DBaseImpl(const Grid2DBaseImpl<NODE, I>& rhs) :
		gridmap(NULL), grid_max_val(rhs.grid_max_val),
		resolution(rhs.resolution)
	{
		init();

		// Copy all of node
		if (rhs.gridmap){
			// TODO: copy operator?
			for (typename OccupancyGridMap::iterator it = rhs.gridmap->begin(); it != rhs.gridmap->end(); it++){
				gridmap->insert(std::pair<Grid2DKey, NODE*>(it->first, new NODE(*(it->second))));
			}
		}
	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::init(){
		this->setResolution(this->resolution);
		for (unsigned i = 0; i < 2; i++){
			max_value[i] = -(std::numeric_limits<double>::max());
			min_value[i] = std::numeric_limits<double>::max();
		}
		size_changed = true;

		// create as many KeyRays as there are OMP_THREADS defined,
		// one buffer for each thread
#ifdef _OPENMP
#pragma omp parallel
#pragma omp critical
		{
			if (omp_get_thread_num() == 0){
				this->keyrays.resize(omp_get_num_threads());
			}

		}
#else
		this->keyrays.resize(1);
#endif

	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::swapContent(Grid2DBaseImpl<NODE, I>& other){
		OccupancyGridMap* this_gridmap = this->gridamp;
		this->gridamp = other.gridamp;
		other.gridamp = this_gridmap;
	}

	/*template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::operator== (const Grid2DBaseImpl<NODE, I>& other) const{
		if (grid_max_val != other.grid_max_val || resolution != other.resolution || this->size() != other.size()){
			return false;
		}

		// traverse all nodes, check if structure the same
		OccupancyGridMap::iterator it = this->gridmap->begin();
		OccupancyGridMap::iterator end = this->gridmap->end();
		OccupancyGridMap::iterator other_it = other.gridmap->begin();
		OccupancyGridMap::iterator other_end = other.gridmap->begin();

		for (; it != end; ++it, ++other_it){
			if (other_it == other_end)
				return false;

			if (it->first != other_it->first
				|| !(*(it->second) == *(other_it->second)))
			{
				return false;
			}
		}

		if(other_it != other_end)
			return false;

		return true;
	}*/

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::setResolution(double r) {
		resolution = r;
		resolution_factor = 1. / resolution;

		grid_center(0) = grid_center(1) = (float)(((double)grid_max_val) / resolution_factor);

		size_changed = true;
	}

	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::coordToKeyChecked(double coordinate, key_type& keyval) const {
		// scale to resolution and shift center for grid_max_val
		int scaled_coord = ((int)floor(resolution_factor * coordinate)) + grid_max_val;

		// keyval within range of grid?
		if ((scaled_coord >= 0) && (((unsigned int)scaled_coord) < (2 * grid_max_val))) {
			keyval = scaled_coord;
			return true;
		}
		return false;
	}

	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::coordToKeyChecked(const point2d& point, Grid2DKey& key) const{
		for (unsigned int i = 0; i < 2; i++) {
			if (!coordToKeyChecked(point(i), key[i])) return false;
		}
		return true;
	}

	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::coordToKeyChecked(double x, double y, Grid2DKey& key) const{
		if (!(coordToKeyChecked(x, key[0]) && coordToKeyChecked(y, key[1]))) {
			return false;
		}
		else {
			return true;
		}
	}

	template <class NODE, class I>
	NODE* Grid2DBaseImpl<NODE, I>::search(const point2d& value) const {
		Grid2DKey key;
		if (!coordToKeyChecked(value, key)){
			GRIDMAP2D_ERROR_STR("Error in search: [" << value << "] is out of Grid2D bounds!");
			return NULL;
		}
		else {
			return this->search(key);
		}

	}

	template <class NODE, class I>
	NODE* Grid2DBaseImpl<NODE, I>::search(double x, double y) const {
		Grid2DKey key;
		if (!coordToKeyChecked(x, y, key)){
			GRIDMAP2D_ERROR_STR("Error in search: [" << x << " " << y << "] is out of Grid2D bounds!");
			return NULL;
		}
		else {
			return this->search(key);
		}
	}


	template <class NODE, class I>
	NODE* Grid2DBaseImpl<NODE, I>::search(const Grid2DKey& key) const {
		if (gridmap->size() == 0)
			return NULL;

		typename OccupancyGridMap::iterator cell = gridmap->find(key);
		if (cell == gridmap->end())
			return NULL;
		return cell->second;
	}

	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::deleteNode(const point2d& value) {
		Grid2DKey key;
		if (!coordToKeyChecked(value, key)){
			GRIDMAP2D_ERROR_STR("Error in deleteNode: [" << value << "] is out of Grid2D bounds!");
			return false;
		}
		else {
			return this->deleteNode(key);
		}
	}

	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::deleteNode(double x, double y) {
		Grid2DKey key;
		if (!coordToKeyChecked(x, y, key)){
			GRIDMAP2D_ERROR_STR("Error in deleteNode: [" << x << " " << y << "] is out of Grid2D bounds!");
			return false;
		}
		else {
			return this->deleteNode(key);
		}
	}


	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::deleteNode(const Grid2DKey& key) {
		if (gridmap->size() == 0)
			return true;

		typename OccupancyGridMap::iterator cell = gridmap->find(key);
		if (cell == gridmap->end())
			return false;

		gridmap->erase(cell);

		return true;
	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::clear() {
		if (gridmap != NULL)
			gridmap->clear();
	}

	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::computeRayKeys(const point2d& origin, const point2d& end, KeyRay& ray) const {

		// see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
		// basically: DDA in 3D

		ray.reset();

		Grid2DKey key_origin, key_end;
		if (!Grid2DBaseImpl<NODE, I>::coordToKeyChecked(origin, key_origin) ||
			!Grid2DBaseImpl<NODE, I>::coordToKeyChecked(end, key_end)) {
			GRIDMAP2D_WARNING_STR("coordinates ( " << origin << " -> " << end << ") out of bounds in computeRayKeys");
			return false;
		}


		if (key_origin == key_end)
			return true; // same grid cell, we're done.

		ray.addKey(key_origin);

		// Initialization phase -------------------------------------------------------

		point2d direction = (end - origin);
		float length = (float)direction.norm();
		direction /= length; // normalize vector

		int    step[2];
		double tMax[2];
		double tDelta[2];

		Grid2DKey current_key = key_origin;

		for (unsigned int i = 0; i < 2; ++i) {
			// compute step direction
			if (direction(i) > 0.0) step[i] = 1;
			else if (direction(i) < 0.0)   step[i] = -1;
			else step[i] = 0;

			// compute tMax, tDelta
			if (step[i] != 0) {
				// corner point of voxel (in direction of ray)
				double voxelBorder = this->keyToCoord(current_key[i]);
				voxelBorder += (float)(step[i] * this->resolution * 0.5);

				tMax[i] = (voxelBorder - origin(i)) / direction(i);
				tDelta[i] = this->resolution / fabs(direction(i));
			}
			else {
				tMax[i] = std::numeric_limits<double>::max();
				tDelta[i] = std::numeric_limits<double>::max();
			}
		}

		// Incremental phase  ---------------------------------------------------------

		bool done = false;
		while (!done) {

			unsigned int dim;

			// find minimum tMax:
			dim = tMax[0] < tMax[1] ? 0 : 1;

			// advance in direction "dim"
			current_key[dim] += step[dim];
			tMax[dim] += tDelta[dim];

			assert(current_key[dim] < 2 * this->grid_max_val);

			// reached endpoint, key equv?
			if (current_key == key_end) {
				done = true;
				break;
			}
			else {

				// reached endpoint world coords?
				// dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
				double dist_from_origin = std::min(tMax[0], tMax[1]);
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

			assert(ray.size() < ray.sizeMax() - 1);

		} // end while

		return true;
	}

	template <class NODE, class I>
	bool Grid2DBaseImpl<NODE, I>::computeRay(const point2d& origin, const point2d& end, std::vector<point2d>& _ray) {
		_ray.clear();
		if (!computeRayKeys(origin, end, keyrays.at(0))) return false;
		for (KeyRay::const_iterator it = keyrays[0].begin(); it != keyrays[0].end(); ++it) {
			_ray.push_back(keyToCoord(*it));
		}
		return true;
	}

	template <class NODE, class I>
	std::ostream& Grid2DBaseImpl<NODE, I>::writeData(std::ostream &s) const{
		if (gridmap){
			size_t node_size = gridmap->size();
			s.write((char*)&node_size, sizeof(node_size));
			Grid2DKey key;
			for (typename OccupancyGridMap::iterator it = gridmap->begin(); it != gridmap->end(); it++){
				// Write key of grid node
				key = it->first;
				s.write((char*)&(key[0]), sizeof(key[0]));
				s.write((char*)&(key[1]), sizeof(key[1]));
				// Write occupancy of grid node
				it->second->writeData(s);
			}
		}

		return s;
	}

	template <class NODE, class I>
	std::istream& Grid2DBaseImpl<NODE, I>::readData(std::istream &s) {
		if (!s.good()){
			GRIDMAP2D_WARNING_STR(__FILE__ << ":" << __LINE__ << "Warning: Input filestream not \"good\"");
		}

		size_t node_size = 0;
		s.read((char*)&node_size, sizeof(node_size));
		GRIDMAP2D_DEBUG_STR("Done (" << node_size << " nodes)");

		size_changed = true;

		// grid needs to be newly created or cleared externally
		if (gridmap->size() != 0) {
			GRIDMAP2D_ERROR_STR("Trying to read into an existing grid.");
			return s;
		}

		if (node_size > 0){
			for (unsigned int i = 0; i < node_size; i++){
				NODE* node;
				Grid2DKey key;
				// Read key of grid node
				s.read((char*)&(key[0]), sizeof(key[0]));
				s.read((char*)&(key[1]), sizeof(key[1]));
				// Read occupancy of grid node
				node = new NODE;
				node->readData(s);

				if (!s.fail()){
					gridmap->insert(std::pair<Grid2DKey, NODE*>(key, node));
				}
				else{
					GRIDMAP2D_ERROR_STR("Grid2DBaseImpl::ReadData: ERROR.\n");
					break;
				}
			}
		}

		return s;
	}

	// non-const versions, 
	// change min/max/size_changed members

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::getMetricSize(double& x, double& y){

		double minX, minY;
		double maxX, maxY;

		getMetricMax(maxX, maxY);
		getMetricMin(minX, minY);

		x = maxX - minX;
		y = maxY - minY;
	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::getMetricSize(double& x, double& y) const{

		double minX, minY;
		double maxX, maxY;

		getMetricMax(maxX, maxY);
		getMetricMin(minX, minY);

		x = maxX - minX;
		y = maxY - minY;
	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::calcMinMax() {
		if (!size_changed)
			return;

		// empty grid
		if (gridmap->size() == 0){
			min_value[0] = min_value[1] = 0.0;
			max_value[0] = max_value[1] = 0.0;
			size_changed = false;
			return;
		}

		for (unsigned i = 0; i < 2; i++){
			max_value[i] = -std::numeric_limits<double>::max();
			min_value[i] = std::numeric_limits<double>::max();
		}

		for (typename Grid2DBaseImpl<NODE, I>::OccupancyGridMap::iterator it = this->gridmap->begin(), end = this->gridmap->end(); it != end; ++it)
		{
			double size = this->resolution;
			double halfSize = size / 2.0;
			double x = keyToCoord(it->first[0]) - halfSize;
			double y = keyToCoord(it->first[1]) - halfSize;
			if (x < min_value[0]) min_value[0] = x;
			if (y < min_value[1]) min_value[1] = y;

			x += size;
			y += size;
			if (x > max_value[0]) max_value[0] = x;
			if (y > max_value[1]) max_value[1] = y;

		}

		size_changed = false;
	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::getMetricMin(double& x, double& y){
		calcMinMax();
		x = min_value[0];
		y = min_value[1];
	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::getMetricMax(double& x, double& y){
		calcMinMax();
		x = max_value[0];
		y = max_value[1];
	}

	// const versions
	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::getMetricMin(double& mx, double& my) const {
		mx = my = std::numeric_limits<double>::max();
		if (size_changed) {
			// empty grid
			if (gridmap->size() == 0){
				mx = my = 0.0;
				return;
			}

			for (typename Grid2DBaseImpl<NODE, I>::OccupancyGridMap::iterator it = this->gridmap->begin(), end = this->gridmap->end(); it != end; ++it) {
				double halfSize = this->resolution / 2.0;
				double x = keyToCoord(it->first[0]) - halfSize;
				double y = keyToCoord(it->first[1]) - halfSize;
				if (x < mx) mx = x;
				if (y < my) my = y;
			}
		} // end if size changed 
		else {
			mx = min_value[0];
			my = min_value[1];
		}
	}

	template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::getMetricMax(double& mx, double& my) const {
		mx = my = -std::numeric_limits<double>::max();
		if (size_changed) {
			// empty grid
			if (gridmap->size() == 0){
				mx = my = 0.0;
				return;
			}

			for (typename Grid2DBaseImpl<NODE, I>::OccupancyGridMap::iterator it = this->gridmap->begin(), end = this->gridmap->end(); it != end; ++it) {
				double halfSize = this->resolution / 2.0;
				double x = keyToCoord(it->first[0]) + halfSize;
				double y = keyToCoord(it->first[1]) + halfSize;
				if (x > mx) mx = x;
				if (y > my) my = y;
			}
		}
		else {
			mx = max_value[0];
			my = max_value[1];
		}
	}

	template <class NODE, class I>
	size_t Grid2DBaseImpl<NODE, I>::memoryUsage() const{
		return (sizeof(Grid2DBaseImpl<NODE, I>) + memoryUsageNode() * this->size());
	}

	// Implement getUnknownLeafCenters - To do.
	/*template <class NODE, class I>
	void Grid2DBaseImpl<NODE, I>::getUnknownLeafCenters(point2d_list& node_centers, point2d pmin, point2d pmax, unsigned int depth) const {

		assert(depth <= tree_depth);
      if (depth == 0)
        depth = tree_depth;

      point3d pmin_clamped = this->keyToCoord(this->coordToKey(pmin, depth), depth);
      point3d pmax_clamped = this->keyToCoord(this->coordToKey(pmax, depth), depth);

      float diff[3];
      unsigned int steps[3];
      float step_size = this->resolution * pow(2, tree_depth-depth);
      for (int i=0;i<3;++i) {
        diff[i] = pmax_clamped(i) - pmin_clamped(i);
        steps[i] = static_cast<unsigned int>(floor(diff[i] / step_size));
        //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
      }

      point3d p = pmin_clamped;
      NODE* res;
      for (unsigned int x=0; x<steps[0]; ++x) {
        p.x() += step_size;
        for (unsigned int y=0; y<steps[1]; ++y) {
          p.y() += step_size;
          for (unsigned int z=0; z<steps[2]; ++z) {
            //          std::cout << "querying p=" << p << std::endl;
            p.z() += step_size;
            res = this->search(p,depth);
            if (res == NULL) {
              node_centers.push_back(p);
            }
          }
          p.z() = pmin_clamped.z();
        }
        p.y() = pmin_clamped.y();
      }
	}*/

	template <class NODE, class I>
	double Grid2DBaseImpl<NODE, I>::volume() {
		double x, y;
		getMetricSize(x, y);
		return x * y;
	}


}
