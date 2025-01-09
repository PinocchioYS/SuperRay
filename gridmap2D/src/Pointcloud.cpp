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

/* According to c++ standard including this header has no practical effect
 * but it can be used to determine the c++ standard library implementation.
 */
#include <ciso646>

#if defined(_MSC_VER) || defined(_LIBCPP_VERSION)
#include <algorithm>
#if ((defined(_MSVC_LANG) && _MSVC_LANG >= 201103L) || __cplusplus >= 201103L)
#include <random>
#endif
#else
#include <ext/algorithm>
#endif
#include <fstream>
#include <math.h>
#include <assert.h>
#include <limits>

#include <superray_gridmap2d/Pointcloud.h>

namespace gridmap2d {
	Pointcloud::Pointcloud() {

	}

	Pointcloud::~Pointcloud() {
		this->clear();
	}

	void Pointcloud::clear() {
		// delete the points
		if (points.size()) {
			points.clear();
		}
	}

	Pointcloud::Pointcloud(const Pointcloud& other) {
		for (Pointcloud::const_iterator it = other.begin(); it != other.end(); it++) {
			points.push_back(point2d(*it));
		}
	}

	Pointcloud::Pointcloud(Pointcloud* other) {
		for (Pointcloud::const_iterator it = other->begin(); it != other->end(); it++) {
			points.push_back(point2d(*it));
		}
	}


	void Pointcloud::push_back(const Pointcloud& other)   {
		for (Pointcloud::const_iterator it = other.begin(); it != other.end(); it++) {
			points.push_back(point2d(*it));
		}
	}

	point2d Pointcloud::getPoint(unsigned int i) const{
		if (i < points.size())
			return points[i];
		else {
			GRIDMAP2D_WARNING("Pointcloud::getPoint index out of range!\n");
			return points.back();
		}
	}

	void Pointcloud::transform(gridmath2d::Pose3D transform) {
		for (unsigned int i = 0; i < points.size(); i++) {
			points[i] = transform.transform(points[i]);
		}

		// FIXME: not correct for multiple transforms
		current_inv_transform = transform.inv();
	}


	void Pointcloud::transformAbsolute(pose3d transform) {
		// undo previous transform, then apply current transform
		pose3d transf = current_inv_transform * transform;

		for (unsigned int i = 0; i < points.size(); i++) {
			points[i] = transf.transform(points[i]);
		}

		current_inv_transform = transform.inv();
	}


	void Pointcloud::rotate(double rot) {
		for (unsigned int i = 0; i < points.size(); i++) {
			points[i].rotate_IP(rot);
		}
	}


	void Pointcloud::calcBBX(point2d& lowerBound, point2d& upperBound) const {
		float min_x, min_y;
		float max_x, max_y;
		min_x = min_y = 1e6;
		max_x = max_y = -1e6;

		float x, y;

		for (Pointcloud::const_iterator it = begin(); it != end(); it++) {
			x = (*it)(0);
			y = (*it)(1);

			if (x < min_x) min_x = x;
			if (y < min_y) min_y = y;

			if (x > max_x) max_x = x;
			if (y > max_y) max_y = y;
		}

		lowerBound(0) = min_x; lowerBound(1) = min_y;
		upperBound(0) = max_x; upperBound(1) = max_y;
	}


	void Pointcloud::crop(point2d lowerBound, point2d upperBound) {

		Pointcloud result;

		float min_x, min_y;
		float max_x, max_y;
		float x, y;

		min_x = lowerBound(0); min_y = lowerBound(1);
		max_x = upperBound(0); max_y = upperBound(1);

		for (Pointcloud::const_iterator it = begin(); it != end(); it++) {
			x = (*it)(0);
			y = (*it)(1);

			if ((x >= min_x) &&
				(y >= min_y) &&
				(x <= max_x) &&
				(y <= max_y)) {
				result.push_back(x, y);
			}
		} // end for points

		this->clear();
		this->push_back(result);
	}


	void Pointcloud::minDist(double thres) {
		Pointcloud result;

		float x, y;
		for (Pointcloud::const_iterator it = begin(); it != end(); it++) {
			x = (*it)(0);
			y = (*it)(1);
			double dist = sqrt(x*x + y*y);
			if (dist > thres) result.push_back(x, y);
		} // end for points
		this->clear();
		this->push_back(result);
	}


	void Pointcloud::subSampleRandom(unsigned int num_samples, Pointcloud& sample_cloud) {
		point2d_collection samples;
		// visual studio does not support random_sample_n and neither does libc++
#if defined(_MSC_VER) || defined(_LIBCPP_VERSION)
		samples.reserve(this->size());
		samples.insert(samples.end(), this->begin(), this->end());
#if ((defined(_MSVC_LANG) && _MSVC_LANG >= 201103L) || __cplusplus >= 201103L)
		std::random_device r;
		std::mt19937 urbg(r());
		std::shuffle(samples.begin(), samples.end(), urbg);
#else
		std::random_shuffle(samples.begin(), samples.end());
#endif
		samples.resize(num_samples);
#else
		random_sample_n(begin(), end(), std::back_insert_iterator<point2d_collection>(samples), num_samples);
		for (unsigned int i=0; i<samples.size(); i++) {
			sample_cloud.push_back(samples[i]);
		}
#endif
	}

	std::istream& Pointcloud::read(std::istream &s){
		while (!s.eof()){
			point2d p;
			for (unsigned int i = 0; i < 2; i++){
				s >> p(i);
			}
			if (!s.fail()){
				this->push_back(p);
			}
			else {
				break;
			}
		}

		return s;
	}

	std::istream& Pointcloud::readBinary(std::istream &s) {
		uint32_t pc_size = 0;
		s.read((char*)&pc_size, sizeof(pc_size));
		GRIDMAP2D_DEBUG("Reading %d points from binary file...", pc_size);

		if (pc_size > 0) {
			this->points.reserve(pc_size);
			point2d p;
			for (uint32_t i = 0; i < pc_size; i++) {
				p.readBinary(s);
				if (!s.fail()) {
					this->push_back(p);
				}
				else {
					GRIDMAP2D_ERROR("Pointcloud::readBinary: ERROR.\n");
					break;
				}
			}
		}
		assert(pc_size == this->size());

		GRIDMAP2D_DEBUG("done.\n");

		return s;
	}


	std::ostream& Pointcloud::writeBinary(std::ostream &s) const {
		// check if written unsigned int can hold size
		size_t orig_size = this->size();
		if (orig_size > std::numeric_limits<uint32_t>::max()){
			GRIDMAP2D_ERROR("Pointcloud::writeBinary ERROR: Point cloud too large to be written");
			return s;
		}

		uint32_t pc_size = static_cast<uint32_t>(this->size());
		GRIDMAP2D_DEBUG("Writing %u points to binary file...", pc_size);
		s.write((char*)&pc_size, sizeof(pc_size));

		for (Pointcloud::const_iterator it = this->begin(); it != this->end(); it++) {
			it->writeBinary(s);
		}
		GRIDMAP2D_DEBUG("done.\n");

		return s;
	}

} // end namespace
