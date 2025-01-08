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

#include <superray_quadmap/math/Vector2.h>
#include <cassert>
#include <math.h>
#include <string.h>

namespace quadmath {

	Vector2& Vector2::rotate_IP(double rot) {
		double x, y;
		// rotate
		x = (*this)(0); y = (*this)(1);
		(*this)(0) = (float)(x * cos(rot) - y * sin(rot));
		(*this)(1) = (float)(x * sin(rot) + y * cos(rot));

		return *this;
	}

	std::istream& Vector2::read(std::istream &s) {
		int temp;
		s >> temp; // should be 2
		for (unsigned int i = 0; i < 2; i++)
			s >> operator()(i);
		return s;
	}


	std::ostream& Vector2::write(std::ostream &s) const {
		s << 2;
		for (unsigned int i = 0; i < 2; i++)
			s << " " << operator()(i);
		return s;
	}

	std::istream& Vector2::readBinary(std::istream &s) {
		int temp;
		s.read((char*)&temp, sizeof(temp));
		double val = 0;
		for (unsigned int i = 0; i < 2; i++) {
			s.read((char*)&val, sizeof(val));
			operator()(i) = (float)val;
		}
		return s;
	}

	std::ostream& Vector2::writeBinary(std::ostream &s) const {
		int temp = 2;
		s.write((char*)&temp, sizeof(temp));
		double val = 0;
		for (unsigned int i = 0; i < 2; i++) {
			val = operator()(i);
			s.write((char*)&val, sizeof(val));
		}
		return s;
	}

	std::ostream& operator<<(std::ostream& out, Vector2 const& v) {
		return out << '(' << v.x() << ' ' << v.y() << ')';
	}

}

