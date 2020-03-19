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

#ifndef GRIDMAP2D_GRID2D_H
#define GRIDMAP2D_GRID2D_H

#include "OccupancyGrid2DBase.h"
#include "Grid2DNode.h"
#include "ScanGraph.h"

namespace gridmap2D {

	/**
	 * gridmap2D main map data structure, stores 2D occupancy grid map.
	 * Basic functionality is implemented in Grid2DBase.
	 *
	 */
	class Grid2D : public OccupancyGrid2DBase <Grid2DNode> {

	public:
		/// Default constructor, sets resolution of leafs
		Grid2D(double resolution);

		/**
         * Reads a Grid2D from a binary file
         * @param _filename
         *
         */
		Grid2D(std::string _filename);

		virtual ~Grid2D(){};

		/// virtual constructor: creates a new object of same type
		/// (Covariant return type requires an up-to-date compiler)
		Grid2D* create() const { return new Grid2D(resolution); }

		std::string getGridType() const { return "Grid2D"; }


	protected:
		/**
		 * Static member object which ensures that this Grid2D's prototype
		 * ends up in the classIDMapping only once. You need this as a
		 * static member in any derived grid2D class in order to read .og2
		 * files through the AbstractGrid2D factory. You should also call
		 * ensureLinking() once from the constructor.
		 */
		class StaticMemberInitializer{
		public:
			StaticMemberInitializer() {
				Grid2D* grid = new Grid2D(0.1);
				grid->clearKeyRays();
				AbstractGrid2D::registerGridType(grid);
			}

			/**
			 * Dummy function to ensure that MSVC does not drop the
			 * StaticMemberInitializer, causing this grid failing to register.
			 * Needs to be called from the constructor of this grid2D.
			 */
			void ensureLinking() {};
		};

		/// to ensure static initialization (only once)
		static StaticMemberInitializer grid2DMemberInit;
	};

} // end namespace

#endif
