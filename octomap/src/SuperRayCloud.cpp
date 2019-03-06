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

#include <algorithm>
#include <fstream>

#include <octomap_superray/SuperRayCloud.h>

namespace octomap{
	SuperRayCloud::SuperRayCloud() {
	}

	SuperRayCloud::~SuperRayCloud() {
		this->clear();
	}

	void SuperRayCloud::clear() {
		// delete the points
		if (superrays.size()) {
			superrays.clear();
		}
	}

	SuperRayCloud::SuperRayCloud(const SuperRayCloud& other) {
		for (SuperRayCloud::const_iterator it = other.begin(); it != other.end(); it++) {
			superrays.push_back(SuperRay(*it));
		}
	}

	SuperRayCloud::SuperRayCloud(SuperRayCloud* other) {
		for (SuperRayCloud::const_iterator it = other->begin(); it != other->end(); it++) {
			superrays.push_back(SuperRay(*it));
		}
	}

	void SuperRayCloud::push_back(const SuperRayCloud& other)   {
		for (SuperRayCloud::const_iterator it = other.begin(); it != other.end(); it++) {
			superrays.push_back(SuperRay(*it));
		}
	}

	SuperRay SuperRayCloud::getPoint(unsigned int i) const{
		if (i < superrays.size())
			return superrays[i];
		else {
			OCTOMAP_WARNING("SuperRayCloud::getPoint index out of range!\n");
			return superrays.back();
		}
	}

	std::ifstream& SuperRayCloud::read(std::ifstream &s){
		// Read origin
		if (!s.eof()){
			point3d o;
			s >> o(0) >> o(1) >> o(2);
			this->origin = o;
		}
		// Read super rays
		while (!s.eof()){
			SuperRay sr;
			s >> sr.p(0) >> sr.p(1) >> sr.p(2) >> sr.w;
			if (!s.fail()){
				this->push_back(sr);
			}
			else {
				break;
			}
		}

		return s;
	}

	std::ofstream& SuperRayCloud::write(std::ofstream &s) const {
		OCTOMAP_DEBUG("Writing %d super rays to binary file...", this->size());
		// Write origin
		s << origin(0) << " " << origin(1) << " " << origin(2) << std::endl;
		// Write super rays
		for (SuperRayCloud::const_iterator it = this->begin(); it != this->end(); it++) {
			s << it->p(0) << " " << it->p(1) << " " << it->p(2) << " " << it->w << std::endl;
		}
		OCTOMAP_DEBUG("done.\n");

		return s;
	}
}
