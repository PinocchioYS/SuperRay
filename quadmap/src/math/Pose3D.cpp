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

#include <quadmap/math/Pose3D.h>
#include <math.h>

namespace quadmath {

	Pose3D::Pose3D() {
	}

	Pose3D::Pose3D(const Vector2 &trans, const double &rot) :
		translation(trans),
		rotation(rot)
	{ }

	Pose3D::Pose3D(float x, float y, double rot) :
		translation(x, y),
		rotation(rot)
	{ }

	Pose3D::~Pose3D() {
	}

	Pose3D& Pose3D::operator= (const Pose3D& other) {
		translation = other.trans();
		rotation = other.rot();
		return *this;
	}

	Pose3D Pose3D::inv() const {
		Pose3D result(*this);
		result.rot() = -rot();
		Vector2 cptrans = this->trans();
		cptrans.rotate_IP(result.rot());
		result.trans() = -cptrans;
		return result;
	}

	Pose3D& Pose3D::inv_IP() {
		rot() = -rot();
		trans().rotate_IP(rot());
		trans() = -trans();
		return *this;
	}


	Vector2 Pose3D::transform(const Vector2 &v) const {
		Vector2 res = v;
		res.rotate_IP(this->rotation);
		res = res + this->trans();
		return res;
	}

	Pose3D Pose3D::operator* (const Pose3D& other) const {
		double rot_new = rotation + other.rot();
		Vector2 trans_new = other.trans();
		trans_new.rotate_IP(rot());
		trans_new = trans_new + trans();
		return Pose3D(trans_new, rot_new);
	}

	const Pose3D& Pose3D::operator*= (const Pose3D& other) {
		Vector2 trans_new = other.trans();
		trans_new.rotate_IP(rot());
		trans_new = trans_new + trans();
		trans() = trans_new;
		rot() = rot() + other.rot();
		return *this;
	}

	double Pose3D::distance(const Pose3D &other) const {
		double dist_x = x() - other.x();
		double dist_y = y() - other.y();
		return sqrt(dist_x*dist_x + dist_y*dist_y);
	}

	double Pose3D::transLength() const {
		return sqrt(x()*x() + y()*y());
	}

	bool Pose3D::operator==(const Pose3D& other) const {
		return translation == other.translation && rotation == other.rotation;
	}

	bool Pose3D::operator!=(const Pose3D& other) const {
		return !(*this == other);
	}

	std::istream& Pose3D::read(std::istream &s) {
		translation.read(s);
		int temp; // should be 1
		s >> temp >> rotation; 
		return s;
	}


	std::ostream& Pose3D::write(std::ostream &s) const {
		translation.write(s);
		s << " " << 1 << " " << rotation;
		return s;
	}


	std::istream& Pose3D::readBinary(std::istream &s) {
		translation.readBinary(s);
		int temp;
		s.read((char*)&temp, sizeof(temp));
		s.read((char*)&rotation, sizeof(rotation));
		return s;
	}


	std::ostream& Pose3D::writeBinary(std::ostream &s) const {
		translation.writeBinary(s);
		int temp = 1;
		s.write((char*)&temp, sizeof(temp));
		s.write((char*)&rotation, sizeof(rotation));
		return s;
	}

	std::ostream& operator<<(std::ostream& s, const Pose3D& p) {
		s << "(" << p.x() << " " << p.y() << ", " << p.rot() << ")";
		return s;
	}

}
