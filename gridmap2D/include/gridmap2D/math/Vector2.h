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

#ifndef GRIDMATH2D_VECTOR2_H
#define GRIDMATH2D_VECTOR2_H

#include <iostream>
#include <math.h>

namespace gridmath2D {

	/*!
	* \brief This class represents a two-dimensional vector
	*/

	class Vector2 {
	public:

		/*!
		* \brief Default constructor
		*/
		Vector2() { data[0] = data[1] = 0.0; }

		/*!
		* \brief Copy constructor
		*
		* @param other a vector of dimension 2
		*/
		Vector2(const Vector2& other) {
			data[0] = other(0);
			data[1] = other(1);
		}

		/*!
		* \brief Constructor
		*
		* Constructs a two-dimensional vector from
		* three single values x, y
		*/
		Vector2(float x, float y) {
			data[0] = x;
			data[1] = y;
		}

		/*!
		* \brief Assignment operator
		*
		* @param other a vector of dimension 2
		*/
		inline Vector2& operator= (const Vector2& other)  {
			data[0] = other(0);
			data[1] = other(1);
			return *this;
		}

		/// dot product
		inline double dot(const Vector2& other) const
		{
			return x()*other.x() + y()*other.y();
		}

		inline const float& operator() (unsigned int i) const
		{
			return data[i];
		}
		inline float& operator() (unsigned int i)
		{
			return data[i];
		}

		inline float& x()
		{
			return operator()(0);
		}

		inline float& y()
		{
			return operator()(1);
		}

		inline const float& x() const
		{
			return operator()(0);
		}

		inline const float& y() const
		{
			return operator()(1);
		}

		inline Vector2 operator- () const
		{
			Vector2 result;
			result(0) = -data[0];
			result(1) = -data[1];
			return result;
		}

		inline Vector2 operator+ (const Vector2 &other) const
		{
			Vector2 result(*this);
			result(0) += other(0);
			result(1) += other(1);
			return result;
		}

		inline Vector2 operator*  (float x) const {
			Vector2 result(*this);
			result(0) *= x;
			result(1) *= x;
			return result;
		}

		inline Vector2 operator- (const Vector2 &other) const
		{
			Vector2 result(*this);
			result(0) -= other(0);
			result(1) -= other(1);
			return result;
		}

		inline void operator+= (const Vector2 &other)
		{
			data[0] += other(0);
			data[1] += other(1);
		}

		inline void operator-= (const Vector2& other) {
			data[0] -= other(0);
			data[1] -= other(1);
		}

		inline void operator/= (float x) {
			data[0] /= x;
			data[1] /= x;
		}

		inline void operator*= (float x) {
			data[0] *= x;
			data[1] *= x;
		}

		inline bool operator== (const Vector2 &other) const {
			for (unsigned int i = 0; i < 2; i++) {
				if (operator()(i) != other(i))
					return false;
			}
			return true;
		}

		/// @return length of the vector ("L2 norm")
		inline double norm() const {
			return sqrt(norm_sq());
		}

		/// @return squared length ("L2 norm") of the vector
		inline double norm_sq() const {
			return (x()*x() + y()*y());
		}

		/// normalizes this vector, so that it has norm=1.0
		inline Vector2& normalize() {
			double len = norm();
			if (len > 0)
				*this /= (float)len;
			return *this;
		}

		/// @return normalized vector, this one remains unchanged
		inline Vector2 normalized() const {
			Vector2 result(*this);
			result.normalize();
			return result;
		}

		inline double angleTo(const Vector2& other) const {
			double dot_prod = this->dot(other);
			double len1 = this->norm();
			double len2 = other.norm();
			return acos(dot_prod / (len1*len2));
		}


		inline double distance(const Vector2& other) const {
			double dist_x = x() - other.x();
			double dist_y = y() - other.y();
			return sqrt(dist_x*dist_x + dist_y*dist_y);
		}

		Vector2& rotate_IP(double rot);

		//    void read (unsigned char * src, unsigned int size);
		std::istream& read(std::istream &s);
		std::ostream& write(std::ostream &s) const;
		std::istream& readBinary(std::istream &s);
		std::ostream& writeBinary(std::ostream &s) const;


	protected:
		float data[2];
	};

	//! user friendly output in format (x y)
	std::ostream& operator<<(std::ostream& out, Vector2 const& v);
}


#endif
