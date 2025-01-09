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

#ifndef GRIDMAP2D_GRID2D_BASE_IMPL_H
#define GRIDMAP2D_GRID2D_BASE_IMPL_H

#include <list>
#include <limits>
#include <iterator>
#include <stack>
#include <bitset>

#include "gridmap2D_types.h"
#include "Grid2DKey.h"
#include "ScanGraph.h"

namespace gridmap2D {

	// forward declaration for NODE
	class AbstractGrid2DNode;

	/**
	 * Grid2D base class, to be used with with any kind of Grid2DDataNode.
	 *
	 * Coordinates values are below +/- 327.68 meters (2^15) at a maximum
	 * resolution of 0.01m.
	 *
	 * This limitation enables the use of an efficient key generation
	 * method which uses the binary representation of the data point
	 * coordinates.
	 *
	 * \note You should probably not use this class directly, but
	 * Grid2DBase or OccupancyGrid2DBase instead
	 *
	 * \tparam NODE Node class to be used in grid (usually derived from
	 *    Grid2DDataNode)
	 * \tparam INTERFACE Interface to be derived from, should be either
	 *    AbstractGrid2D or AbstractOccupancyGrid2D
	 */
	template <class NODE, class INTERFACE>
	class Grid2DBaseImpl : public INTERFACE {

	public:
		/// Make the templated NODE type available from the outside
		typedef NODE NodeType;

		Grid2DBaseImpl(double resolution);
		virtual ~Grid2DBaseImpl();

		/// Deep copy constructor
		Grid2DBaseImpl(const Grid2DBaseImpl<NODE, INTERFACE>& rhs);


		/**
		 * Swap contents of two grids, i.e., only the underlying
		 * pointer. You have to ensure yourself that the
		 * metadata (resolution etc) matches. No memory is cleared
		 * in this function
		 */
		void swapContent(Grid2DBaseImpl<NODE, INTERFACE>& rhs);

		/// Comparison between two grids, all meta data, all
		/// nodes, and the structure must be identical
//		bool operator== (const Grid2DBaseImpl<NODE, INTERFACE>& rhs) const;

		std::string getGridType() const { return "Grid2DBaseImpl"; }

		/// Change the resolution of the grid2D, scaling all voxels.
		/// This will not preserve the (metric) scale!
		void setResolution(double r);
		inline double getResolution() const { return resolution; }

		/**
		 * Clear KeyRay vector to minimize unneeded memory. This is only
		 * useful for the StaticMemberInitializer classes, don't call it for
		 * a grid2D that is actually used.
		 */
		void clearKeyRays(){
			keyrays.clear();
		}

		/**
		* \return Pointer to the grid. This pointer
		* should not be modified or deleted externally, the Grid2D
		* manages its memory itself.
		*/
		typedef unordered_ns::unordered_map<Grid2DKey, NODE*, Grid2DKey::KeyHash> OccupancyGridMap;
		inline OccupancyGridMap* getGrid() const { return gridmap; }

		/**
		 *  Search node given a 2d point (x, y).
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(double x, double y) const;

		/**
		 *  Search node given a 2d point (point2d).
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(const point2d& value) const;

		/**
		 *  Search a node given an addressing key
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(const Grid2DKey& key) const;

		/**
		 *  Delete a node (if exists) given a 2d point (x, y).
		 */
		bool deleteNode(double x, double y);

		/**
		 *  Delete a node (if exists) given a 2d point (point2d).
		 */
		bool deleteNode(const point2d& value);

		/**
		 *  Delete a node (if exists) given an addressing key.
		 */
		bool deleteNode(const Grid2DKey& key);

		/// Deletes the complete grid structure
		void clear();

		// -- statistics  ----------------------

		/// \return The number of nodes in the grid
		virtual inline size_t size() const { return gridmap->size(); }

		/// \return Memory usage of the grid2D in bytes (may vary between architectures)
		virtual size_t memoryUsage() const;     // TODO: add the memory usage of hash table

		/// \return Memory usage of a single grid2D node
		virtual inline size_t memoryUsageNode() const { return sizeof(NODE); }

		double volume();

		/// Size of Grid2D (all known space) in meters for x and y dimension
		virtual void getMetricSize(double& x, double& y);
		/// Size of Grid2D (all known space) in meters for x and y dimension
		virtual void getMetricSize(double& x, double& y) const;
		/// minimum value of the bounding box of all known space in x, y
		virtual void getMetricMin(double& x, double& y);
		/// minimum value of the bounding box of all known space in x, y
		void getMetricMin(double& x, double& y) const;
		/// maximum value of the bounding box of all known space in x, y
		virtual void getMetricMax(double& x, double& y);
		/// maximum value of the bounding box of all known space in x, y
		void getMetricMax(double& x, double& y) const;

		/// return centers of leafs that do NOT exist (but could) in a given bounding box
		// void getUnknownLeafCenters(point2d_list& node_centers, point2d pmin, point2d pmax, unsigned int depth = 0) const; - To do.


		// -- raytracing  -----------------------

		/**
		 * Traces a ray from origin to end (excluding), returning an
		 * Grid2DKey of all nodes traversed by the beam. You still need to check
		 * if a node at that coordinate exists (e.g. with search()).
		 *
		 * @param origin start coordinate of ray
		 * @param end end coordinate of ray
		 * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
		 * @return Success of operation. Returning false usually means that one of the coordinates is out of the Grid2D's range
		 */
		bool computeRayKeys(const point2d& origin, const point2d& end, KeyRay& ray) const;


		/**
		 * Traces a ray from origin to end (excluding), returning the
		 * coordinates of all nodes traversed by the beam. You still need to check
		 * if a node at that coordinate exists (e.g. with search()).
		 * @note: use the faster computeRayKeys method if possible.
		 *
		 * @param origin start coordinate of ray
		 * @param end end coordinate of ray
		 * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
		 * @return Success of operation. Returning false usually means that one of the coordinates is out of the Grid2D's range
		 */
		bool computeRay(const point2d& origin, const point2d& end, std::vector<point2d>& ray);


		// file IO

		/**
		 * Read all nodes from the input stream (without file header),
		 * for this the grid needs to be already created.
		 * For general file IO, you
		 * should probably use AbstractGrid2D::read() instead.
		 */
		std::istream& readData(std::istream &s);

		/// Write complete state of grid to stream (without file header) unmodified.
		std::ostream& writeData(std::ostream &s) const;

		/// @return beginning of the grid as iterator
//		const OccupancyGridMap::iterator begin() const { return gridmap->begin(); }
		/// @return end of the grid as iterator
//		const OccupancyGridMap::iterator end() const { return gridmap->end(); }

		//
		// Key / coordinate conversion functions
		//

		/// Converts from a single coordinate into a discrete key
		inline key_type coordToKey(double coordinate) const{
			return ((int)floor(resolution_factor * coordinate)) + grid_max_val;
		}

		/// Converts from a 2D coordinate into a 2D addressing key
		inline Grid2DKey coordToKey(const point2d& coord) const{
			return Grid2DKey(coordToKey(coord(0)), coordToKey(coord(1)));
		}

		/// Converts from a 2D coordinate into a 2D addressing key
		inline Grid2DKey coordToKey(double x, double y) const{
			return Grid2DKey(coordToKey(x), coordToKey(y));
		}

		/**
		 * Converts a 2D coordinate into a 2D Grid2DKey, with boundary checking.
		 *
		 * @param coord 2d coordinate of a point
		 * @param key values that will be computed, an array of fixed size 2.
		 * @return true if point is within the grid2D (valid), false otherwise
		 */
		bool coordToKeyChecked(const point2d& coord, Grid2DKey& key) const;

		/**
		 * Converts a 2D coordinate into a 2D Grid2DKey, with boundary checking.
		 *
		 * @param x
		 * @param y
		 * @param key values that will be computed, an array of fixed size 2.
		 * @return true if point is within the grid2D (valid), false otherwise
		 */
		bool coordToKeyChecked(double x, double y, Grid2DKey& key) const;

		/**
		 * Converts a single coordinate into a discrete addressing key, with boundary checking.
		 *
		 * @param coordinate 2d coordinate of a point
		 * @param key discrete 16 bit adressing key, result
		 * @return true if coordinate is within the grid2D bounds (valid), false otherwise
		 */
		bool coordToKeyChecked(double coordinate, key_type& key) const;

		/// converts from a discrete key into a coordinate corresponding to the key's center
		inline double keyToCoord(key_type key) const{
			return (double((int)key - (int) this->grid_max_val) + 0.5) * this->resolution;
		}

		/// converts from an addressing key into a coordinate corresponding to the key's center
		inline point2d keyToCoord(const Grid2DKey& key) const{
			return point2d(float(keyToCoord(key[0])), float(keyToCoord(key[1])));
		}

	protected:
		/// Constructor to enable derived classes to change grid constants.
		/// This usually requires a re-implementation of some core grid-traversal functions as well!
		Grid2DBaseImpl(double resolution, unsigned int grid_max_val);

		/// initialize non-trivial members, helper for constructors
		void init();

		/// recalculates min and max in x, y. Does nothing when grid size didn't change.
		void calcMinMax();

	private:
		/// Assignment operator is private: don't (re-)assign grid2D
		/// (const-parameters can't be changed) -  use the copy constructor instead.
		Grid2DBaseImpl<NODE, INTERFACE>& operator=(const Grid2DBaseImpl<NODE, INTERFACE>&);

	protected:
		OccupancyGridMap* gridmap;

		// constants of the grid
		const unsigned int grid_max_val;
		double resolution;  ///< in meters
		double resolution_factor; ///< = 1. / resolution

		/// flag to denote whether the grid2D extent changed (for lazy min/max eval)
		bool size_changed;

		point2d grid_center;  // coordinate offset of grid

		double max_value[2]; ///< max in x, y
		double min_value[2]; ///< min in x, y

		/// data structure for ray casting, array for multithreading
		std::vector<KeyRay> keyrays;
	};

}

#include "Grid2DBaseImpl.hxx"

#endif
