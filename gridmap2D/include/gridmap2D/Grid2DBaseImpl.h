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

	// forward declaration for NODE children array
	class AbstractGrid2DNode;

	/**
	 * OcTree base class, to be used with with any kind of OcTreeDataNode.
	 *
	 * This tree implementation currently has a maximum depth of 16
	 * nodes. For this reason, coordinates values have to be, e.g.,
	 * below +/- 327.68 meters (2^15) at a maximum resolution of 0.01m.
	 *
	 * This limitation enables the use of an efficient key generation
	 * method which uses the binary representation of the data point
	 * coordinates.
	 *
	 * \note You should probably not use this class directly, but
	 * QuadTreeBase or OccupancyQuadTreeBase instead
	 *
	 * \tparam NODE Node class to be used in tree (usually derived from
	 *    QuadTreeDataNode)
	 * \tparam INTERFACE Interface to be derived from, should be either
	 *    AbstractQuadTree or AbstractOccupancyQuadTree
	 */
	template <class NODE, class INTERFACE>
	class Grid2DBaseImpl : public INTERFACE {

	public:
		/// Make the templated NODE type available from the outside
		typedef NODE NodeType;

		// the actual iterator implementation is included here
		// as a member from this file
// #include <gridmap2D/Grid2DIterator.hxx>

		Grid2DBaseImpl(double resolution);
		virtual ~Grid2DBaseImpl();

		/// Deep copy constructor
		Grid2DBaseImpl(const Grid2DBaseImpl<NODE, INTERFACE>& rhs);


		/**
		 * Swap contents of two gridmaps, i.e., only the underlying
		 * pointer / tree structure. You have to ensure yourself that the
		 * metadata (resolution etc) matches. No memory is cleared
		 * in this function
		 */
		void swapContent(Grid2DBaseImpl<NODE, INTERFACE>& rhs);

		/// Comparison between two octrees, all meta data, all
		/// nodes, and the structure must be identical
		// bool operator== (const Grid2DBaseImpl<NODE, INTERFACE>& rhs) const;

		std::string getGridType() const { return "Grid2DBaseImpl"; }

		/// Change the resolution of the quadtree, scaling all voxels.
		/// This will not preserve the (metric) scale!
		void setResolution(double r);
		inline double getResolution() const { return resolution; }

		// inline double getNodeSize(unsigned depth) const { assert(depth <= tree_depth); return sizeLookupTable[depth]; }

		/**
		 * Clear KeyRay vector to minimize unneeded memory. This is only
		 * useful for the StaticMemberInitializer classes, don't call it for
		 * an quadtree that is actually used.
		 */
		void clearKeyRays(){
			keyrays.clear();
		}

		/**
		 *  Search node at specified depth given a 2d point.
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(double x, double y) const;

		/**
		 *  Search node at specified depth given a 2d point (depth=0: search full tree depth)
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(const point2d& value) const;

		/**
		 *  Search a node at specified depth given an addressing key (depth=0: search full tree depth)
		 *  You need to check if the returned node is NULL, since it can be in unknown space.
		 *  @return pointer to node if found, NULL otherwise
		 */
		NODE* search(const Grid2DKey& key) const;

		/**
		 *  Delete a node (if exists) given a 2d point. Will always
		 *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
		 *  Pruned nodes at level "depth" will directly be deleted as a whole.
		 */
		bool deleteNode(double x, double y);

		/**
		 *  Delete a node (if exists) given a 2d point. Will always
		 *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
		 *  Pruned nodes at level "depth" will directly be deleted as a whole.
		 */
		bool deleteNode(const point2d& value);

		/**
		 *  Delete a node (if exists) given an addressing key. Will always
		 *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
		 *  Pruned nodes at level "depth" will directly be deleted as a whole.
		 */
		bool deleteNode(const Grid2DKey& key);

		/// Deletes the complete tree structure
		void clear();

		// -- statistics  ----------------------

		/// \return The number of nodes in the tree
		virtual inline size_t size() const { return gridmap->size(); }

		/// \return Memory usage of the complete octree in bytes (may vary between architectures)
		virtual size_t memoryUsage() const;	// Add HashTable?

		/// \return Memory usage of a single octree node
		virtual inline size_t memoryUsageNode() const { return sizeof(NODE); };

		double volume();

		/// Size of QuadTree (all known space) in meters for x and y dimension
		virtual void getMetricSize(double& x, double& y);
		/// Size of QuadTree (all known space) in meters for x and y dimension
		virtual void getMetricSize(double& x, double& y) const;
		/// minimum value of the bounding box of all known space in x, y
		virtual void getMetricMin(double& x, double& y);
		/// minimum value of the bounding box of all known space in x, y
		void getMetricMin(double& x, double& y) const;
		/// maximum value of the bounding box of all known space in x, y
		virtual void getMetricMax(double& x, double& y);
		/// maximum value of the bounding box of all known space in x, y
		void getMetricMax(double& x, double& y) const;

		// -- access tree nodes  ------------------

		/// return centers of leafs that do NOT exist (but could) in a given bounding box
		// void getUnknownLeafCenters(point2d_list& node_centers, point2d pmin, point2d pmax, unsigned int depth = 0) const; - To do.


		// -- raytracing  -----------------------

		/**
		 * Traces a ray from origin to end (excluding), returning an
		 * QuadTreeKey of all nodes traversed by the beam. You still need to check
		 * if a node at that coordinate exists (e.g. with search()).
		 *
		 * @param origin start coordinate of ray
		 * @param end end coordinate of ray
		 * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
		 * @return Success of operation. Returning false usually means that one of the coordinates is out of the QuadTree's range
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
		 * @return Success of operation. Returning false usually means that one of the coordinates is out of the QuadTree's range
		 */
		bool computeRay(const point2d& origin, const point2d& end, std::vector<point2d>& ray);


		// file IO

		/**
		 * Read all nodes from the input stream (without file header),
		 * for this the tree needs to be already created.
		 * For general file IO, you
		 * should probably use AbstractQuadTree::read() instead.
		 */
		std::istream& readData(std::istream &s); // - To do.

		/// Write complete state of tree to stream (without file header) unmodified.
		/// Pruning the tree first produces smaller files (lossless compression)
		std::ostream& writeData(std::ostream &s) const;  // - To do.

//		typedef OccupancyGridMap::iterator Map_iterator;

		/// @return beginning of the tree as leaf iterator
//		const OccupancyGridMap::iterator begin() const { return gridmap->begin(); }
		/// @return end of the tree as leaf iterator
//		const OccupancyGridMap::iterator end() const { return gridmap->end(); } // TODO: RVE?

		/// @return beginning of the tree as leaf iterator
		// leaf_iterator begin_leafs(unsigned char maxDepth = 0) const { return leaf_iterator(this, maxDepth); };
		/// @return end of the tree as leaf iterator
		// const leaf_iterator end_leafs() const { return leaf_iterator_end; }

		/// @return beginning of the tree as leaf iterator in a bounding box
		/*leaf_bbx_iterator begin_leafs_bbx(const QuadTreeKey& min, const QuadTreeKey& max, unsigned char maxDepth = 0) const {
			return leaf_bbx_iterator(this, min, max, maxDepth);
		}*/
		/// @return beginning of the tree as leaf iterator in a bounding box
		/*leaf_bbx_iterator begin_leafs_bbx(const point2d& min, const point2d& max, unsigned char maxDepth = 0) const {
			return leaf_bbx_iterator(this, min, max, maxDepth);
		}*/
		/// @return end of the tree as leaf iterator in a bounding box
		// const leaf_bbx_iterator end_leafs_bbx() const { return leaf_iterator_bbx_end; }

		/// @return beginning of the tree as iterator to all nodes (incl. inner)
		// tree_iterator begin_tree(unsigned char maxDepth = 0) const { return tree_iterator(this, maxDepth); }
		/// @return end of the tree as iterator to all nodes (incl. inner)
		// const tree_iterator end_tree() const { return tree_iterator_end; }*/

		//
		// Key / coordinate conversion functions
		//

		/// Converts from a single coordinate into a discrete key
		inline key_type coordToKey(double coordinate) const{
			return ((int)floor(resolution_factor * coordinate)) + grid_max_val;
		}

		/// Converts from a single coordinate into a discrete key at a given depth
		// key_type coordToKey(double coordinate, unsigned depth) const;


		/// Converts from a23D coordinate into a 2D addressing key
		inline Grid2DKey coordToKey(const point2d& coord) const{
			return Grid2DKey(coordToKey(coord(0)), coordToKey(coord(1)));
		}

		/// Converts from a 2D coordinate into a 2D addressing key
		inline Grid2DKey coordToKey(double x, double y) const{
			return Grid2DKey(coordToKey(x), coordToKey(y));
		}

		/**
		 * Converts a 2D coordinate into a 2D OcTreeKey, with boundary checking.
		 *
		 * @param coord 2d coordinate of a point
		 * @param key values that will be computed, an array of fixed size 2.
		 * @return true if point is within the quadtree (valid), false otherwise
		 */
		bool coordToKeyChecked(const point2d& coord, Grid2DKey& key) const;

		/**
		 * Converts a 2D coordinate into a 2D OcTreeKey, with boundary checking.
		 *
		 * @param x
		 * @param y
		 * @param key values that will be computed, an array of fixed size 2.
		 * @return true if point is within the quadtree (valid), false otherwise
		 */
		bool coordToKeyChecked(double x, double y, Grid2DKey& key) const;

		/**
		 * Converts a single coordinate into a discrete addressing key, with boundary checking.
		 *
		 * @param coordinate 2d coordinate of a point
		 * @param key discrete 16 bit adressing key, result
		 * @return true if coordinate is within the quadtree bounds (valid), false otherwise
		 */
		bool coordToKeyChecked(double coordinate, key_type& key) const;

		/// converts from a discrete key at the lowest tree level into a coordinate
		/// corresponding to the key's center
		inline double keyToCoord(key_type key) const{
			return (double((int)key - (int) this->grid_max_val) + 0.5) * this->resolution;
		}

		/// converts from an addressing key at the lowest tree level into a coordinate
		/// corresponding to the key's center
		inline point2d keyToCoord(const Grid2DKey& key) const{
			return point2d(float(keyToCoord(key[0])), float(keyToCoord(key[1])));
		}

	protected:
		/// Constructor to enable derived classes to change tree constants.
		/// This usually requires a re-implementation of some core tree-traversal functions as well!
		Grid2DBaseImpl(double resolution, unsigned int grid_max_val);

		/// initialize non-trivial members, helper for constructors
		void init();

		/// recalculates min and max in x, y. Does nothing when tree size didn't change.
		void calcMinMax();

	private:
		/// Assignment operator is private: don't (re-)assign quadtrees
		/// (const-parameters can't be changed) -  use the copy constructor instead.
		// QuadTreeBaseImpl<NODE, INTERFACE>& operator=(const QuadTreeBaseImpl<NODE, INTERFACE>&);

	protected:
		// void allocNodeChildren(NODE* node);

		// NODE* root; ///< Pointer to the root NODE, NULL for empty tree
		typedef unordered_ns::unordered_map<Grid2DKey, NODE*, Grid2DKey::KeyHash> OccupancyGridMap;
		OccupancyGridMap* gridmap;

		// constants of the tree
		const unsigned int grid_max_val;
		double resolution;  ///< in meters
		double resolution_factor; ///< = 1. / resolution

		// size_t grid_size; ///< number of nodes in tree ( does not need, use gridmap->size();
		/// flag to denote whether the quadtree extent changed (for lazy min/max eval)
		bool size_changed;

		point2d grid_center;  // coordinate offset of tree

		double max_value[2]; ///< max in x, y
		double min_value[2]; ///< min in x, y

		/// data structure for ray casting, array for multithreading
		std::vector<KeyRay> keyrays;

		// const leaf_iterator leaf_iterator_end;
		// const leaf_bbx_iterator leaf_iterator_bbx_end;
		// const tree_iterator tree_iterator_end;
	};

}

#include <gridmap2D/Grid2DBaseImpl.hxx>

#endif
