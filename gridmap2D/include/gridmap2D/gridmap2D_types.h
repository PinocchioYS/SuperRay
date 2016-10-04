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

#ifndef GRIDMAP2D_TYPES_H
#define GRIDMAP2D_TYPES_H

#include <stdio.h>
#include <vector>
#include <list>

#include <gridmap2D/math/Vector2.h>
#include <gridmap2D/math/Pose3D.h>
#include <gridmap2D/gridmap2D_deprecated.h>

namespace gridmap2D {

	/// Use Vector2 (float precision) as a point2d in quadmap
	typedef gridmath2D::Vector2               point2d;
	/// Use our Pose3D (float precision) as pose3d in quadmap
	typedef gridmath2D::Pose3D                pose3d;

	typedef std::vector<gridmath2D::Vector2>  point2d_collection;
	typedef std::list<gridmath2D::Vector2>    point2d_list;

	/// A voxel defined by its center point2d and its side length
	typedef std::pair<point2d, double>		QuadTreeVolume;

}

// no debug output if not in debug mode:
#ifdef NDEBUG
#ifndef GRIDMAP2D_NODEBUGOUT
#define GRIDMAP2D_NODEBUGOUT
#endif
#endif

#ifdef GRIDMAP2D_NODEBUGOUT
#define GRIDMAP2D_DEBUG(...)       (void)0
#define GRIDMAP2D_DEBUG_STR(...)   (void)0
#else
#define GRIDMAP2D_DEBUG(...)        fprintf(stderr, __VA_ARGS__), fflush(stderr)
#define GRIDMAP2D_DEBUG_STR(args)   std::cerr << args << std::endl
#endif

#define GRIDMAP2D_WARNING(...)      fprintf(stderr, "WARNING: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
#define GRIDMAP2D_WARNING_STR(args) std::cerr << "WARNING: " << args << std::endl
#define GRIDMAP2D_ERROR(...)        fprintf(stderr, "ERROR: "), fprintf(stderr, __VA_ARGS__), fflush(stderr)
#define GRIDMAP2D_ERROR_STR(args)   std::cerr << "ERROR: " << args << std::endl

#endif
