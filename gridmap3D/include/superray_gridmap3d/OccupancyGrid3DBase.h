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

#ifndef GRIDMAP3D_OCCUPANCY_GRID3D_BASE_H
#define GRIDMAP3D_OCCUPANCY_GRID3D_BASE_H

#include <list>
#include <stdlib.h>
#include <vector>

#include "gridmap3d_types.h"
#include "gridmap3d_utils.h"
#include "Grid3DBaseImpl.h"
#include "AbstractOccupancyGrid3D.h"


namespace gridmap3d {

	/**
	 * Base implementation for Occupancy Grid3Ds (e.g. for mapping).
	 * AbstractOccupancyGrid3D serves as a common
	 * base interface for all these classes.
	 * Each class used as NODE type needs to be derived from
	 * OccupancyGrid3DNode.
	 *
	 * At a resolution of 1 cm, values have to be < +/- 327.68 meters (2^15)
	 *
	 * This limitation enables the use of an efficient key generation
	 * method which uses the binary representation of the data.
	 *
	 * \tparam NODE Node class to be used in grid (usually derived from
	 *    Grid3DDataNode)
	 */
	template <class NODE>
	class OccupancyGrid3DBase : public Grid3DBaseImpl<NODE, AbstractOccupancyGrid3D> {

	public:
		/// Default constructor, sets resolution of leafs
		OccupancyGrid3DBase(double resolution);
		virtual ~OccupancyGrid3DBase();

		/// Copy constructor
		OccupancyGrid3DBase(const OccupancyGrid3DBase<NODE>& rhs);

		/**
		 * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
		 * This function simply inserts all rays of the point clouds as batch operation.
		 * Discretization effects can lead to the deletion of occupied space, it is
		 * usually recommended to use insertPointCloud() instead.
		 *
		 * @param scan Pointcloud (measurement endpoints), in global reference frame
		 * @param sensor_origin measurement origin in global reference frame
		 * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
		 */
		virtual void insertPointCloudRays(const Pointcloud& scan, const point3d& sensor_origin, double maxrange = -1.);

		/**
		 * Set log_odds value of voxel to log_odds_value. 
		 *
		 * @param key Grid3DKey of the NODE that is to be updated
		 * @param log_odds_value value to be set as the log_odds value of the node
		 * @return pointer to the updated NODE
		 */
		virtual NODE* setNodeValue(const Grid3DKey& key, float log_odds_value);

		/**
		 * Set log_odds value of voxel to log_odds_value.
		 * Looks up the Grid3DKey corresponding to the coordinate and then calls setNodeValue() with it.
		 *
		 * @param value 3d coordinate of the NODE that is to be updated
		 * @param log_odds_value value to be set as the log_odds value of the node
		 * @return pointer to the updated NODE
		 */
		virtual NODE* setNodeValue(const point3d& value, float log_odds_value);

		/**
		 * Set log_odds value of voxel to log_odds_value.
		 * Looks up the Grid3DKey corresponding to the coordinate and then calls setNodeValue() with it.
		 *
		 * @param x
		 * @param y
		 * @param z
		 * @param log_odds_value value to be set as the log_odds value of the node
		 * @return pointer to the updated NODE
		 */
		virtual NODE* setNodeValue(double x, double y, double z, float log_odds_value);

		/**
		 * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
		 *
		 * @param key Grid3DKey of the NODE that is to be updated
		 * @param log_odds_update value to be added (+) to log_odds value of node
		 * @return pointer to the updated NODE
		 */
		virtual NODE* updateNode(const Grid3DKey& key, float log_odds_update);

		/**
		 * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
		 * Looks up the Grid3DKey corresponding to the coordinate and then calls updateNode() with it.
		 *
		 * @param value 3d coordinate of the NODE that is to be updated
		 * @param log_odds_update value to be added (+) to log_odds value of node
		 * @return pointer to the updated NODE
		 */
		virtual NODE* updateNode(const point3d& value, float log_odds_update);

		/**
		 * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
		 * Looks up the Grid3DKey corresponding to the coordinate and then calls updateNode() with it.
		 *
		 * @param x
		 * @param y
		 * @param z
		 * @param log_odds_update value to be added (+) to log_odds value of node
		 * @return pointer to the updated NODE
		 */
		virtual NODE* updateNode(double x, double y, double z, float log_odds_update);

		/**
		 * Integrate occupancy measurement.
		 *
		 * @param key Grid3DKey of the NODE that is to be updated
		 * @param occupied true if the node was measured occupied, else false
		 * @return pointer to the updated NODE
		 */
		virtual NODE* updateNode(const Grid3DKey& key, bool occupied);

		/**
		 * Integrate occupancy measurement.
		 * Looks up the Grid3DKey corresponding to the coordinate and then calls udpateNode() with it.
		 *
		 * @param value 3d coordinate of the NODE that is to be updated
		 * @param occupied true if the node was measured occupied, else false
		 * @return pointer to the updated NODE
		 */
		virtual NODE* updateNode(const point3d& value, bool occupied);

		/**
		 * Integrate occupancy measurement.
		 * Looks up the Grid3DKey corresponding to the coordinate and then calls udpateNode() with it.
		 *
		 * @param x
		 * @param y
		 * @param z
		 * @param occupied true if the node was measured occupied, else false
		 * @return pointer to the updated NODE
		 */
		virtual NODE* updateNode(double x, double y, double z, bool occupied);


		/**
		 * Creates the maximum likelihood map by calling toMaxLikelihood on all
		 * grid nodes, setting their occupancy to the corresponding occupancy thresholds.
		 */
		virtual void toMaxLikelihood();

		/**
		 * Insert one ray between origin and end into the grid.
		 * integrateMissOnRay() is called for the ray, the end point is updated as occupied.
		 * It is usually more efficient to insert complete pointclouds with insertPointCloud() or
		 * insertPointCloudRays().
		 *
		 * @param origin origin of sensor in global coordinates
		 * @param end endpoint of measurement in global coordinates
		 * @param maxrange maximum range after which the raycast should be aborted
		 * @return success of operation
		 */
		virtual bool insertRay(const point3d& origin, const point3d& end, double maxrange = -1.0);

		/**
		 * Performs raycasting in 3d, similar to computeRay(). Can be called in parallel e.g. with OpenMP
		 * for a speedup.
		 *
		 * A ray is cast from 'origin' with a given direction, the first non-free
		 * cell is returned in 'end' (as center coordinate). This could also be the
		 * origin node if it is occupied or unknown. castRay() returns true if an occupied node
		 * was hit by the raycast. If the raycast returns false you can search() the node at 'end' and
		 * see whether it's unknown space.
		 *
		 * @param[in] origin starting coordinate of ray
		 * @param[in] direction A vector pointing in the direction of the raycast (NOT a point in space). Does not need to be normalized.
		 * @param[out] end returns the center of the last cell on the ray. If the function returns true, it is occupied.
		 * @param[in] ignoreUnknownCells whether unknown cells are ignored (= treated as free). If false (default), the raycast aborts when an unknown cell is hit and returns false.
		 * @param[in] maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
		 * @return true if an occupied cell was hit, false if the maximum range or grid bounds are reached, or if an unknown node was hit.
		 */
		virtual bool castRay(const point3d& origin, const point3d& direction, point3d& end,
			bool ignoreUnknownCells = false, double maxRange = -1.0) const;

		/**
		 * Retrieves the entry point of a ray into a voxel. This is the closest intersection point of the ray
		 * originating from origin and a plane of the axis aligned cube.
		 *
		 * @param[in] origin Starting point of ray
		 * @param[in] direction A vector pointing in the direction of the raycast. Does not need to be normalized.
		 * @param[in] center The center of the voxel where the ray terminated. This is the output of castRay.
		 * @param[out] intersection The entry point of the ray into the voxel, on the voxel surface.
		 * @param[in] delta A small increment to avoid ambiguity of beeing exactly on a voxel surface. A positive value will get the point out of the hit voxel, while a negative valuewill get it inside.
		 * @return Whether or not an intesection point has been found. Either, the ray never cross the voxel or the ray is exactly parallel to the only surface it intersect.
		 */
		virtual bool getRayIntersection(const point3d& origin, const point3d& direction, const point3d& center,
			point3d& intersection, double delta = 0.0) const;

		//-- set BBX limit (limits grid updates to this bounding box)

		///  use or ignore BBX limit (default: ignore)
		void useBBXLimit(bool enable) { use_bbx_limit = enable; }
		bool bbxSet() const { return use_bbx_limit; }
		/// sets the minimum for a query bounding box to use
		void setBBXMin(const point3d& min);
		/// sets the maximum for a query bounding box to use
		void setBBXMax(const point3d& max);
		/// @return the currently set minimum for bounding box queries, if set
		point3d getBBXMin() const { return bbx_min; }
		/// @return the currently set maximum for bounding box queries, if set
		point3d getBBXMax() const { return bbx_max; }
		point3d getBBXBounds() const;
		point3d getBBXCenter() const;
		/// @return true if point is in the currently set bounding box
		bool inBBX(const point3d& p) const;
		/// @return true if key is in the currently set bounding box
		bool inBBX(const Grid3DKey& key) const;

		//-- change detection on occupancy:
		/// track or ignore changes while inserting scans (default: ignore)
		void enableChangeDetection(bool enable) { use_change_detection = enable; }
		bool isChangeDetectionEnabled() const { return use_change_detection; }
		/// Reset the set of changed keys. Call this after you obtained all changed nodes.
		void resetChangeDetection() { changed_keys.clear(); }

		/**
		 * Iterator to traverse all keys of changed nodes.
		 * you need to enableChangeDetection() first.
		 */
		KeyBoolMap::const_iterator changedKeysBegin() const { return changed_keys.begin(); }

		/// Iterator to traverse all keys of changed nodes.
		KeyBoolMap::const_iterator changedKeysEnd() const { return changed_keys.end(); }

		/// Number of changes since last reset.
		size_t numChangesDetected() const { return changed_keys.size(); }

		// -- I/O  -----------------------------------------

		/**
         * Reads only the data (=complete grid structure) from the input stream.
         * The grid needs to be constructed with the proper header information
         * beforehand, see readBinary().
         */
		std::istream& readBinaryData(std::istream &s);

		/**
         * Writes the data of the grid (without header) to the stream.
         */
		std::ostream& writeBinaryData(std::ostream &s) const;

		/// integrate a "hit" measurement according to the grid's sensor model
		virtual void integrateHit(NODE* occupancyNode) const;
		/// integrate a "miss" measurement according to the grid's sensor model
		virtual void integrateMiss(NODE* occupancyNode) const;
		/// update logodds value of node by adding to the current value.
		virtual void updateNodeLogOdds(NODE* occupancyNode, const float& update) const;

		/// converts the node to the maximum likelihood value according to the grid's parameter for "occupancy"
		virtual void nodeToMaxLikelihood(NODE* occupancyNode) const;
		/// converts the node to the maximum likelihood value according to the grid's parameter for "occupancy"
		virtual void nodeToMaxLikelihood(NODE& occupancyNode) const;

	protected:
		/// Constructor to enable derived classes to change grid constants.
		/// This usually requires a re-implementation of some core grid-traversal functions as well!
		OccupancyGrid3DBase(double resolution, unsigned int grid_max_val);

		/**
		 * Traces a ray from origin to end and updates all voxels on the
		 *  way as free.  The volume containing "end" is not updated.
		 */
		inline bool integrateMissOnRay(const point3d& origin, const point3d& end);

	protected:
		bool use_bbx_limit;  ///< use bounding box for queries (needs to be set)?
		point3d bbx_min;
		point3d bbx_max;
		Grid3DKey bbx_min_key;
		Grid3DKey bbx_max_key;

		bool use_change_detection;
		/// Set of keys which changed since last resetChangeDetection
		KeyBoolMap changed_keys;
	};

} // namespace

#include "OccupancyGrid3DBase.hxx"

#endif
