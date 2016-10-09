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

#ifndef GRIDMAP2D_GRID2D_NODE_H
#define GRIDMAP2D_GRID2D_NODE_H

#include "gridmap2D_types.h"
#include "gridmap2D_utils.h"
#include "Grid2DDataNode.h"
#include <limits>

namespace gridmap2D {

	/**
	 * Nodes to be used in Grid2D. They represent 2d occupancy grid cells.
	 * "value" stores their log-odds occupancy.
	 */
	class Grid2DNode : public Grid2DDataNode<float> {

	public:
		Grid2DNode();
		~Grid2DNode();

		// -- node occupancy  ----------------------------

		/// \return occupancy probability of node
		inline double getOccupancy() const { return probability(value); }

		/// \return log odds representation of occupancy probability of node
		inline float getLogOdds() const{ return value; }
		/// sets log odds occupancy of node
		inline void setLogOdds(float l) { value = l; }

		/// adds p to the node's logOdds value (with no boundary / threshold checking!)
		void addValue(const float& p);

	protected:
		// "value" stores log odds occupancy probability
	};

} // end namespace

#endif
