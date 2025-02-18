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

#ifndef GRIDMAP2D_GRID2D_DATA_NODE_H
#define GRIDMAP2D_GRID2D_DATA_NODE_H

#include "gridmap2d_types.h"
#include "assert.h"

namespace gridmap2d {

	class AbstractGrid2DNode {


	};

	// forward declaration for friend in Grid2DDataNode
	template<typename NODE, typename I> class Grid2DBaseImpl;

	/**
	 * Basic node in the Grid2D that can hold arbitrary data of type T in value.
	 * This is the base class for nodes used in a Grid2D. The used implementation
	 * for occupancy mapping is in Grid2DNode.#
	 * \tparam T data to be stored in the node (e.g. a float for probabilities)
	 */
	template<typename T> class Grid2DDataNode : public AbstractGrid2DNode {
		template<typename NODE, typename I>
		friend class Grid2DBaseImpl;

	public:

		Grid2DDataNode();
		Grid2DDataNode(T initVal);

		/// Copy constructor
		Grid2DDataNode(const Grid2DDataNode& rhs);

		/// Delete only own members.
		~Grid2DDataNode();

		/// Copy the payload (data in "value") from rhs into this node
		void copyData(const Grid2DDataNode& from);

		/// Equals operator, compares if the stored value is identical
		bool operator==(const Grid2DDataNode& rhs) const;

		/// @return value stored in the node
		T getValue() const{ return value; }
		/// sets value to be stored in the node
		void setValue(T v) { value = v; }

		// file IO:

		/// Read node payload (data only) from binary stream
		std::istream& readData(std::istream &s);

		/// Write node payload (data only) to binary stream
		std::ostream& writeData(std::ostream &s) const;


		/// Make the templated data type available from the outside
		typedef T DataType;


	protected:
		/// stored data (payload)
		T value;

	};


} // end namespace

#include "Grid2DDataNode.hxx"

#endif
