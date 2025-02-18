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

namespace gridmap3d {

	template <typename T>
	Grid3DDataNode<T>::Grid3DDataNode()
	{
	}

	template <typename T>
	Grid3DDataNode<T>::Grid3DDataNode(T initVal)
		: value(initVal)
	{
	}

	template <typename T>
	Grid3DDataNode<T>::Grid3DDataNode(const Grid3DDataNode<T>& rhs)
		: value(rhs.value)
	{
	}

	template <typename T>
	Grid3DDataNode<T>::~Grid3DDataNode()
	{
	}

	template <typename T>
	void Grid3DDataNode<T>::copyData(const Grid3DDataNode<T>& from){
		value = from.value;
	}

	template <typename T>
	bool Grid3DDataNode<T>::operator== (const Grid3DDataNode<T>& rhs) const{
		return rhs.value == value;
	}

	// ============================================================
	// =  File IO           =======================================
	// ============================================================

	template <typename T>
	std::istream& Grid3DDataNode<T>::readData(std::istream &s) {
		s.read((char*)&value, sizeof(value));
		return s;
	}


	template <typename T>
	std::ostream& Grid3DDataNode<T>::writeData(std::ostream &s) const{
		s.write((const char*)&value, sizeof(value));
		return s;
	}

} // end namespace

