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

#ifndef QUADMATH_POSE3D_H
#define QUADMATH_POSE3D_H

#include "Vector2.h"

namespace quadmath {

	/*!
	 * \brief This class represents a two-dimensional pose of an object
	 * 
	 * The two-dimensional pose is represented by a two-dimensional
	 * translation vector representing the position of the object and
	 * a rotation value representing the rotation angle of the object.
	 */
	class Pose3D {
	public:

		Pose3D();
		~Pose3D();

		/*!
		 * \brief Constructor
		 *
		 * Constructs a pose from given translation and rotation.
		 */
		Pose3D(const Vector2& trans, const double& rot);

		/*!
		 * \brief Constructor
		 *
		 * Constructs a pose from a translation represented by
		 * its x, y-values and a rotation
		 */
		Pose3D(float x, float y, double r);

		/*!
		 * \brief Constructor
		 *
		 * Constructs a pose from another pose
		 */
		Pose3D(const Pose3D& other);
		Pose3D& operator= (const Pose3D& other);

		bool operator==(const Pose3D& other) const;
		bool operator!=(const Pose3D& other) const;

		/*!
		 * \brief Translational component
		 *
		 * @return the translational component of this pose
		 */
		inline Vector2& trans() { return translation; }

		/*!
		 * \brief Rotational component
		 *
		 * @return the rotational component of this pose
		 */
		inline double& rot() { return rotation; }

		/*!
		 * \brief Translational component
		 *
		 * @return the translational component of this pose
		 */
		const Vector2& trans() const { return translation; }

		/*!
		 * \brief Rotational component
		 * @return the rotational component of this pose
		 */
		const double& rot() const { return rotation; }

		inline float& x() { return translation(0); }
		inline float& y() { return translation(1); }
		inline double& r() { return rotation; }
		inline const float& x() const { return translation(0); }
		inline const float& y() const { return translation(1); }
		inline const double& r() const { return rotation; }

		/*!
		 * \brief Transformation of a vector
		 *
		 * Transforms the vector v by the transformation which is
		 * specified by this.
		 * @return the vector which is translated by the translation of
		 * this and afterwards rotated by the rotation of this.
		 */
		Vector2 transform(const Vector2 &v) const;

		/*!
		 * \brief Inversion
		 *
		 * Inverts the coordinate transformation represented by this pose
		 * @return a copy of this pose inverted
		 */
		Pose3D inv() const;

		/*!
		 * \brief Inversion
		 *
		 * Inverts the coordinate transformation represented by this pose
		 * @return a reference to this pose
		 */
		Pose3D& inv_IP();

		/*!
		 * \brief Concatenation
		 *
		 * Concatenates the coordinate transformations represented
		 * by this and p.
		 * @return this * p (applies first this, then p)
		 */
		Pose3D operator* (const Pose3D &p) const;

		/*!
		 * \brief In place concatenation
		 *
		 * Concatenates p to this Pose6D.
		 * @return this which results from first moving by this and
		 * afterwards by p
		 */
		const Pose3D& operator*= (const Pose3D &p);

		/*!
		 * \brief Translational distance
		 *
		 * @return the translational (euclidian) distance to p
		 */
		double distance(const Pose3D &other) const;

		/*!
		 * \brief Translational length
		 *
		 * @return the translational (euclidian) length of the translation
		 * vector of this Pose6D
		 */
		double transLength() const;

		/*!
		 * \brief Output operator
		 *
		 * Output to stream in a format which can be parsed using read().
		 */
		std::ostream& write(std::ostream &s) const;
		/*!
		 * \brief Input operator
		 *
		 * Parsing from stream which was written by write().
		 */
		std::istream& read(std::istream &s);
		/*!
		 * \brief Binary output operator
		 *
		 * Output to stream in a binary format which can be parsed using readBinary().
		 */
		std::ostream& writeBinary(std::ostream &s) const;
		/*!
		 * \brief Binary input operator
		 *
		 * Parsing from binary stream which was written by writeBinary().
		 */
		std::istream& readBinary(std::istream &s);

	protected:
		Vector2 translation;
		double	rotation;
	};

	//! user friendly output in format (x y, r) which is (translation, rotation)
	std::ostream& operator<<(std::ostream& s, const Pose3D& p);

}

#endif
