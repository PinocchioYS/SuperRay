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

#ifndef GRIDMAP2D_GRID2D_KEY_H
#define GRIDMAP2D_GRID2D_KEY_H

/* According to c++ standard including this header has no practical effect
 * but it can be used to determine the c++ standard library implementation.
 */
#include <ciso646>

#include <assert.h>

/* Libc++ does not implement the TR1 namespace, all c++11 related functionality
 * is instead implemented in the std namespace.
 */
#if defined(__GNUC__) && ! defined(_LIBCPP_VERSION)
#include <tr1/unordered_set>
#include <tr1/unordered_map>
namespace gridmap2D {
	namespace unordered_ns = std::tr1;
};
#else
#include <unordered_set>
#include <unordered_map>
namespace gridmap2D {
	namespace unordered_ns = std;
}
#endif

#include <vector>

namespace gridmap2D {

	typedef uint16_t key_type;

	/**
	 * Grid2DKey is a container class for internal key addressing. The keys count the
	 * number of cells (voxels) from the origin as discrete address of a voxel.
	 * @see Grid2DBaseImpl::coordToKey() and Grid2DBaseImpl::keyToCoord() for conversions.
	 */
	class Grid2DKey {

	public:
		Grid2DKey() {}
		Grid2DKey(key_type a, key_type b){
			k[0] = a;
			k[1] = b;
		}

		Grid2DKey(const Grid2DKey& other){
			k[0] = other.k[0];
			k[1] = other.k[1];
		}

		bool operator== (const Grid2DKey &other) const {
			return ((k[0] == other[0]) && (k[1] == other[1]));
		}

		bool operator!= (const Grid2DKey& other) const {
			return((k[0] != other[0]) || (k[1] != other[1]));
		}

		Grid2DKey& operator=(const Grid2DKey& other){
			k[0] = other.k[0]; k[1] = other.k[1];
			return *this;
		}

		const key_type& operator[] (unsigned int i) const {
			return k[i];
		}

		key_type& operator[] (unsigned int i) {
			return k[i];
		}

		key_type k[2];

		/// Provides a hash function on Keys
		struct KeyHash{
			std::size_t operator()(const Grid2DKey& key) const{
				// a simple hashing function 
				// explicit casts to size_t to operate on the complete range
				// constanst will be promoted according to C++ standard
				return static_cast<size_t>(key.k[0])
					+ 1447 * static_cast<size_t>(key.k[1]);
			}
		};

	};

	/**
	 * Data structure to efficiently compute the nodes to update from a scan
	 * insertion using a hash set.
	 * @note you need to use boost::unordered_set instead if your compiler does not
	 * yet support tr1!
	 */
	typedef unordered_ns::unordered_set<Grid2DKey, Grid2DKey::KeyHash> KeySet;

	/**
	 * Data structrure to efficiently track changed nodes as a combination of
	 * Grid2DKeys and a bool flag (to denote newly created nodes)
	 *
	 */
	typedef unordered_ns::unordered_map<Grid2DKey, bool, Grid2DKey::KeyHash> KeyBoolMap;

	class KeyRay {
	public:

		KeyRay() {
			ray.resize(maxSize);
			reset();
		}

		KeyRay(const KeyRay& other){
			ray = other.ray;
			std::size_t dSize = other.end() - other.begin();
			end_of_ray = ray.begin() + dSize;
		}

		void reset() {
			end_of_ray = begin();
		}

		void addKey(const Grid2DKey& k) {
			assert(end_of_ray != ray.end());
			*end_of_ray = k;
			++end_of_ray;
		}

		size_t size() const { return end_of_ray - ray.begin(); }
		size_t sizeMax() const { return maxSize; }

		typedef std::vector<Grid2DKey>::iterator iterator;
		typedef std::vector<Grid2DKey>::const_iterator const_iterator;
		typedef std::vector<Grid2DKey>::reverse_iterator reverse_iterator;

		iterator begin() { return ray.begin(); }
		iterator end() { return end_of_ray; }
		const_iterator begin() const { return ray.begin(); }
		const_iterator end() const   { return end_of_ray; }

		reverse_iterator rbegin() { return (reverse_iterator)end_of_ray; }
		reverse_iterator rend() { return ray.rend(); }

	private:
		std::vector<Grid2DKey> ray;
		std::vector<Grid2DKey>::iterator end_of_ray;
		const static size_t maxSize = 100000;
	};

} // namespace

#endif
