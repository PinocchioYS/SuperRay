/*
* Copyright(c) 2016, Youngsun Kwon, Donghyuk Kim, and Sung-eui Yoon, KAIST
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met :
*
*     * Redistributions of source code must retain the above copyright notice, this
*       list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright notice,
*       this list of conditions and the following disclaimer in the documentation
*       and / or other materials provided with the distribution.
*     * Neither the name of SuperRay nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef GRIDMAP3D_SUPERRAY_SUPERRAY_CLOUD_H
#define GRIDMAP3D_SUPERRAY_SUPERRAY_CLOUD_H

#include <vector>
#include <list>
#include "SuperRay.h"

/**
* A collection of super rays with different weights
*/
namespace gridmap3d {
	class SuperRayCloud {
	public:
		SuperRayCloud();
		~SuperRayCloud();

		SuperRayCloud(const SuperRayCloud& other);
		SuperRayCloud(SuperRayCloud* other);

		size_t size() const { return superrays.size(); }
		void clear();
		inline void reserve(size_t size) { superrays.reserve(size); }

		inline void push_back(float x, float y, float z, int w) {
			superrays.push_back(SuperRay(x, y, z, w));
		}
		inline void push_back(const point3d& p, int w) {
			superrays.push_back(SuperRay(p, w));
		}
		inline void push_back(const SuperRay& sr) {
			superrays.push_back(sr);
		}

		/// Add superrays from other SuperRayCloud
		void push_back(const SuperRayCloud& other);

		// iterators ------------------

		typedef std::vector<SuperRay>::iterator iterator;
		typedef std::vector<SuperRay>::const_iterator const_iterator;
		iterator begin() { return superrays.begin(); }
		iterator end()   { return superrays.end(); }
		const_iterator begin() const { return superrays.begin(); }
		const_iterator end() const  { return superrays.end(); }
		SuperRay back()  { return superrays.back(); }
		/// Returns a copy of the ith point in point cloud.
		/// Use operator[] for direct access to point reference.
		SuperRay getPoint(unsigned int i) const;

		inline const SuperRay& operator[] (size_t i) const { return superrays[i]; }
		inline SuperRay& operator[] (size_t i) { return superrays[i]; }

		// I/O methods

		std::ifstream& read(std::ifstream &s);
		std::ofstream& write(std::ofstream& s) const;

		std::vector<SuperRay>	superrays;
		point3d		origin;
	};
}


#endif
