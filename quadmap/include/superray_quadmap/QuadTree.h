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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OW NER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QUADMAP_QUADTREE_H
#define QUADMAP_QUADTREE_H


#include "OccupancyQuadTreeBase.h"
#include "QuadTreeNode.h"
#include "ScanGraph.h"

namespace quadmap {

	/**
	 * quadmap main map data structure, stores 2D occupancy grid map in a QuadTree.
	 * Basic functionality is implemented in QuadTreeBase.
	 *
	 */
	class QuadTree : public OccupancyQuadTreeBase <QuadTreeNode> {

	public:
		/// Default constructor, sets resolution of leafs
		QuadTree(double resolution);

		/**
		 * Reads a QuadTree from a binary file
		 * @param _filename
		 *
		 */
		QuadTree(std::string _filename);

		virtual ~QuadTree(){}

		/// virtual constructor: creates a new object of same type
		/// (Covariant return type requires an up-to-date compiler)
		QuadTree* create() const { return new QuadTree(resolution); }

		std::string getTreeType() const { return "QuadTree"; }


	protected:
		/**
		 * Static member object which ensures that this QuadTree's prototype
		 * ends up in the classIDMapping only once. You need this as a
		 * static member in any derived quadtree class in order to read .ot2
		 * files through the AbstractQuadTree factory. You should also call
		 * ensureLinking() once from the constructor.
		 */
		class StaticMemberInitializer{
		public:
			StaticMemberInitializer() {
				QuadTree* tree = new QuadTree(0.1);
				tree->clearKeyRays();
				AbstractQuadTree::registerTreeType(tree);
			}

			/**
			 * Dummy function to ensure that MSVC does not drop the
			 * StaticMemberInitializer, causing this tree failing to register.
			 * Needs to be called from the constructor of this quadtree.
			 */
			void ensureLinking() {}
		};

		/// to ensure static initialization (only once)
		static StaticMemberInitializer quadTreeMemberInit;
	};

} // end namespace

#endif
