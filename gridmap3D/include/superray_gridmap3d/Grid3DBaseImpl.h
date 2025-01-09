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

#ifndef GRIDMAP3D_GRID3D_BASE_IMPL_H
#define GRIDMAP3D_GRID3D_BASE_IMPL_H

#include <list>
#include <limits>
#include <iterator>
#include <stack>
#include <bitset>

#include "gridmap3d_types.h"
#include "Grid3DKey.h"
#include "ScanGraph.h"

namespace gridmap3d {

	// forward declaration for NODE
	class AbstractGrid3DNode;

	/**
	 * Grid3D base class, to be used with with any kind of Grid3DDataNode.
	 *
	 * Coordinates values are below +/- 327.68 meters (2^15) at a maximum 
	 * resolution of 0.01m.
	 *
	 * This limitation enables the use of an efficient key generation
	 * method which uses the binary representation of the data point
	 * coordinates.
	 *
	 * \note You should probably not use this class directly, but
	 * Grid3DBase or OccupancyGrid3DBase instead
	 *
	 * \tparam NODE Node class to be used in grid (usually derived from
	 *    Grid3DDataNode)
	 * \tparam INTERFACE Interface to be derived from, should be either
	 *    AbstractGrid3D or AbstractOccupancyGrid3D
	 */
	template <class NODE, class INTERFACE>
	class Grid3DBaseImpl : public INTERFACE {

	public:
		/// Make the templated NODE type available from the outside
		typedef NODE NodeType;

		Grid3DBaseImpl(double resolution);
		virtual ~Grid3DBaseImpl();

		/// Deep copy constructor
		Grid3DBaseImpl(const Grid3DBaseImpl<NODE, INTERFACE>& rhs);


		/**
		 * Swap contents of two grids, i.e., only the underlying
		 * pointer. You have to ensure yourself that the
		 * metadata (resolution etc) matches. No memory is cleared
		 * in this function
		 */
		void swapContent(Grid3DBaseImpl<NODE, INTERFACE>& rhs);

		/// Comparison between two grids, all meta data, all
		/// nodes, and the structure must be identical
//		bool operator== (const Grid3DBaseImpl<NODE, INTERFACE>& rhs) const;

		std::string getGridType() const { return "Grid3DBaseImpl"; }

		/// Change the resolution of the grid3D, scaling all voxels.
		/// This will not preserve the (metric) scale!
		void setResolution(double r);
		inline double getResolution() const { return resolution; }

		/**
		 * Clear KeyRay vector to minimize unneeded memory. This is only
		 * useful for the StaticMemberInitializer classes, don't call it for
		 * a grid3D that is actually used.
		 */
		void clearKeyRays(){
			keyrays.clear();
		}

		/**
		* \return Pointer to the grid. This pointer
		* should not be modified or deleted externally, the Grid3D
		* manages its memory itself.
		*/
		typedef unordered_ns::unordered_map<Grid3DKey, NODE*, Grid3DKey::KeyHash> OccupancyGridMap;
		inline OccupancyGridMap* getGrid() const { return gridmap; }

		/**
		 *  Search node given a 3d point (x, y, z).
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(double x, double y, double z) const;

		/**
		 *  Search node given a 3d point (point3d).
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(const point3d& value) const;

		/**
		 *  Search a node given an addressing key 
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(const Grid3DKey& key) const;

		/**
		 *  Delete a node (if exists) given a 3d point (x, y, z). 
		 */
		bool deleteNode(double x, double y, double z);

		/**
		 *  Delete a node (if exists) given a 3d point (point3d). 
		 */
		bool deleteNode(const point3d& value);

		/**
		 *  Delete a node (if exists) given an addressing key. 
		 */
		bool deleteNode(const Grid3DKey& key);

		/// Deletes the complete grid structure
		void clear();

		// -- statistics  ----------------------

		/// \return The number of nodes in the grid
		virtual inline size_t size() const { return gridmap->size(); }

		/// \return Memory usage of the grid3D in bytes (may vary between architectures)
		virtual size_t memoryUsage() const;     // TODO: add the memory usage of hash table

		/// \return Memory usage of a single grid3D node
		virtual inline size_t memoryUsageNode() const { return sizeof(NODE); }

		double volume();

		/// Size of Grid3D (all known space) in meters for x, y and z dimension
		virtual void getMetricSize(double& x, double& y, double& z);
		/// Size of Grid3D (all known space) in meters for x, y and z dimension
		virtual void getMetricSize(double& x, double& y, double& z) const;
		/// minimum value of the bounding box of all known space in x, y, z
		virtual void getMetricMin(double& x, double& y, double& z);
		/// minimum value of the bounding box of all known space in x, y, z
		void getMetricMin(double& x, double& y, double& z) const;
		/// maximum value of the bounding box of all known space in x, y, z
		virtual void getMetricMax(double& x, double& y, double& z);
		/// maximum value of the bounding box of all known space in x, y, z
		void getMetricMax(double& x, double& y, double& z) const;

		/// return centers of leafs that do NOT exist (but could) in a given bounding box
		// void getUnknownLeafCenters(point3d_list& node_centers, point3d pmin, point3d pmax, unsigned int depth = 0) const; - To do.


		// -- raytracing  -----------------------

		/**
		 * Traces a ray from origin to end (excluding), returning an
		 * Grid3DKey of all nodes traversed by the beam. You still need to check
		 * if a node at that coordinate exists (e.g. with search()).
		 *
		 * @param origin start coordinate of ray
		 * @param end end coordinate of ray
		 * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
		 * @return Success of operation. Returning false usually means that one of the coordinates is out of the Grid3D's range
		 */
		bool computeRayKeys(const point3d& origin, const point3d& end, KeyRay& ray) const;


		/**
		 * Traces a ray from origin to end (excluding), returning the
		 * coordinates of all nodes traversed by the beam. You still need to check
		 * if a node at that coordinate exists (e.g. with search()).
		 * @note: use the faster computeRayKeys method if possible.
		 *
		 * @param origin start coordinate of ray
		 * @param end end coordinate of ray
		 * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
		 * @return Success of operation. Returning false usually means that one of the coordinates is out of the Grid3D's range
		 */
		bool computeRay(const point3d& origin, const point3d& end, std::vector<point3d>& ray);


		// file IO

		/**
		 * Read all nodes from the input stream (without file header),
		 * for this the grid needs to be already created.
		 * For general file IO, you
		 * should probably use AbstractGrid3D::read() instead.
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

		/// Converts from a 3D coordinate into a 3D addressing key
		inline Grid3DKey coordToKey(const point3d& coord) const{
			return Grid3DKey(coordToKey(coord(0)), coordToKey(coord(1)), coordToKey(coord(2)));
		}

		/// Converts from a 3D coordinate into a 3D addressing key
		inline Grid3DKey coordToKey(double x, double y, double z) const{
			return Grid3DKey(coordToKey(x), coordToKey(y), coordToKey(z));
		}

		/**
		 * Converts a 3D coordinate into a 3D Grid3DKey, with boundary checking.
		 *
		 * @param coord 3d coordinate of a point
		 * @param key values that will be computed, an array of fixed size 3.
		 * @return true if point is within the grid3D (valid), false otherwise
		 */
		bool coordToKeyChecked(const point3d& coord, Grid3DKey& key) const;

		/**
		 * Converts a 3D coordinate into a 3D Grid3DKey, with boundary checking.
		 *
		 * @param x
		 * @param y
         * @param z
		 * @param key values that will be computed, an array of fixed size 3.
		 * @return true if point is within the grid3D (valid), false otherwise
		 */
		bool coordToKeyChecked(double x, double y, double z, Grid3DKey& key) const;

		/**
		 * Converts a single coordinate into a discrete addressing key, with boundary checking.
		 *
		 * @param coordinate 3d coordinate of a point
		 * @param key discrete 16 bit adressing key, result
		 * @return true if coordinate is within the grid3D bounds (valid), false otherwise
		 */
		bool coordToKeyChecked(double coordinate, key_type& key) const;

		/// converts from a discrete key into a coordinate corresponding to the key's center
		inline double keyToCoord(key_type key) const{
			return (double((int)key - (int) this->grid_max_val) + 0.5) * this->resolution;
		}

		/// converts from an addressing key into a coordinate corresponding to the key's center
		inline point3d keyToCoord(const Grid3DKey& key) const{
			return point3d(float(keyToCoord(key[0])), float(keyToCoord(key[1])), float(keyToCoord(key[2])));
		}

	protected:
		/// Constructor to enable derived classes to change grid constants.
		/// This usually requires a re-implementation of some core grid-traversal functions as well!
		Grid3DBaseImpl(double resolution, unsigned int grid_max_val);

		/// initialize non-trivial members, helper for constructors
		void init();

		/// recalculates min and max in x, y. Does nothing when grid size didn't change.
		void calcMinMax();

	private:
		/// Assignment operator is private: don't (re-)assign grid3D
		/// (const-parameters can't be changed) -  use the copy constructor instead.
		Grid3DBaseImpl<NODE, INTERFACE>& operator=(const Grid3DBaseImpl<NODE, INTERFACE>&);

	protected:
		OccupancyGridMap* gridmap;

		// constants of the grid
		const unsigned int grid_max_val;
		double resolution;  ///< in meters
		double resolution_factor; ///< = 1. / resolution

		/// flag to denote whether the grid extent changed (for lazy min/max eval)
		bool size_changed;

		point3d grid_center;  // coordinate offset of grid

		double max_value[3]; ///< max in x, y, z
		double min_value[3]; ///< min in x, y, z

		/// data structure for ray casting, array for multithreading
		std::vector<KeyRay> keyrays;
	};

}

#include "Grid3DBaseImpl.hxx"

#endif
