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

#include <superray_quadmap/SuperRayQuadTree.h>

namespace quadmap{
	SuperRayQuadTree::SuperRayQuadTree(double in_resolution)
	: OccupancyQuadTreeBase<QuadTreeNode>(in_resolution) {
		superrayQuadTreeMemberInit.ensureLinking();
	};

	SuperRayQuadTree::SuperRayQuadTree(std::string _filename)
			: OccupancyQuadTreeBase<QuadTreeNode>(0.1)  { // resolution will be set according to tree file
		readBinary(_filename);
	}

	SuperRayQuadTree::StaticMemberInitializer SuperRayQuadTree::superrayQuadTreeMemberInit;

	void SuperRayQuadTree::insertPointCloudRays(const Pointcloud& pc, const point2d& origin)
	{
		if (pc.size() < 1)
			return;

	#ifdef _OPENMP
		omp_set_num_threads(this->keyrays.size());
	#pragma omp parallel for
	#endif
		for (int i = 0; i < (int)pc.size(); ++i) {
			const point2d& p = pc[i];
			unsigned threadIdx = 0;
	#ifdef _OPENMP
			threadIdx = omp_get_thread_num();
	#endif
			KeyRay* keyray = &(this->keyrays.at(threadIdx));

			// free cells
			if (this->computeRayKeys(origin, p, *keyray)){
	#ifdef _OPENMP
	#pragma omp critical
	#endif
				{
					for (KeyRay::iterator it = keyray->begin(); it != keyray->end(); it++) {
						updateNode(*it, false); // insert freespace measurement
					}
				}
			}
		}

		for (int i = 0; i < (int)pc.size(); ++i){
			updateNode(pc[i], true); // update endpoint to be occupied
		}
	}

	void SuperRayQuadTree::insertSuperRayCloudRays(const Pointcloud& scan, const point2d& origin, const int threshold)
	{
		SuperRayGenerator srgenerator(resolution, tree_max_val, threshold);
		SuperRayCloud srcloud;
		srgenerator.GenerateSuperRay(scan, origin, srcloud);
		insertSuperRayCloudRays(srcloud);
	}

	void SuperRayQuadTree::insertSuperRayCloudRays(const SuperRayCloud& superray)
	{
		if (superray.size() < 1)
			return;

		point2d origin = superray.origin;
	#ifdef _OPENMP
		omp_set_num_threads(this->keyrays.size());
	#pragma omp parallel
	#endif
		for (int i = 0; i < (int)superray.size(); ++i) {
			const point2d& p = superray[i].p;
			const float& missprob = prob_miss_log * superray[i].w;
			unsigned threadIdx = 0;
	#ifdef _OPENMP
			threadIdx = omp_get_thread_num();
	#endif
			KeyRay* keyray = &(this->keyrays.at(threadIdx));

			// free cells
			if (this->computeRayKeys(origin, p, *keyray)){
	#ifdef _OPENMP
	#pragma omp critical
	#endif
				{
					for (KeyRay::iterator it = keyray->begin(); it != keyray->end(); it++) {
						updateNode(*it, missprob, false);
					}
				}
			}
		}

		for (int i = 0; i < (int)superray.size(); ++i){
			updateNode(superray[i].p, prob_hit_log * superray[i].w, false);
		}
	}
}