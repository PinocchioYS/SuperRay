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

#ifndef GRIDMAP2D_SUPERRAY_SUPERRAY_H
#define GRIDMAP2D_SUPERRAY_SUPERRAY_H

#include <gridmap2D/gridmap2D_types.h>

namespace gridmap2D{
	class SuperRay{
	public:
		SuperRay(void) : p(0, 0), w(0) {}
		SuperRay(float _x, float _y, int _w) : p(_x, _y), w(_w) {}
		SuperRay(point2d _p, int _w) : p(_p), w(_w) {}
		SuperRay(const SuperRay& _other) : p(_other.p), w(_other.w) {}
		~SuperRay(void) {}

		point2d p;
		int		w;
	};
}

#endif
