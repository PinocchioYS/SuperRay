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


#include <superray_gridmap3d/AbstractOccupancyGrid3D.h>
#include <superray_gridmap3d/gridmap3d_types.h>

namespace gridmap3d {
	AbstractOccupancyGrid3D::AbstractOccupancyGrid3D(){
		// some sane default values:
		setOccupancyThres(0.5);   // = 0.0 in logodds
		setProbHit(0.7);          // = 0.85 in logodds
		setProbMiss(0.4);         // = -0.4 in logodds

		setClampingThresMin(0.1192); // = -2 in log odds
		setClampingThresMax(0.971); // = 3.5 in log odds
	}

	bool AbstractOccupancyGrid3D::writeBinary(const std::string& filename){
		std::ofstream binary_outfile(filename.c_str(), std::ios_base::binary);

		if (!binary_outfile.is_open()){
			GRIDMAP3D_ERROR_STR("Filestream to " << filename << " not open, nothing written.");
			return false;
		}
		return writeBinary(binary_outfile);
	}

	bool AbstractOccupancyGrid3D::writeBinaryConst(const std::string& filename) const{
		std::ofstream binary_outfile(filename.c_str(), std::ios_base::binary);

		if (!binary_outfile.is_open()){
			GRIDMAP3D_ERROR_STR("Filestream to " << filename << " not open, nothing written.");
			return false;
		}
		writeBinaryConst(binary_outfile);
		binary_outfile.close();
		return true;
	}

	bool AbstractOccupancyGrid3D::writeBinary(std::ostream &s){
		// convert to max likelihood
		this->toMaxLikelihood();
		return writeBinaryConst(s);
	}

	bool AbstractOccupancyGrid3D::writeBinaryConst(std::ostream &s) const{
		// write new header first:
		s << binaryFileHeader << "\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
		s << "id " << this->getGridType() << std::endl;
		s << "size " << this->size() << std::endl;
		s << "res " << this->getResolution() << std::endl;
		s << "data" << std::endl;

		writeBinaryData(s);

		if (s.good()){
			GRIDMAP3D_DEBUG(" done.\n");
			return true;
		}
		else {
			GRIDMAP3D_WARNING_STR("Output stream not \"good\" after writing grid");
			return false;
		}
	}

	bool AbstractOccupancyGrid3D::readBinary(const std::string& filename){
		std::ifstream binary_infile(filename.c_str(), std::ios_base::binary);
		if (!binary_infile.is_open()){
			GRIDMAP3D_ERROR_STR("Filestream to " << filename << " not open, nothing read.");
			return false;
		}
		return readBinary(binary_infile);
	}

	bool AbstractOccupancyGrid3D::readBinary(std::istream &s) {
		if (!s.good()){
			GRIDMAP3D_WARNING_STR("Input filestream not \"good\" in Grid3D::readBinary");
		}

		// check if first line valid:
		std::string line;
		std::getline(s, line);
		unsigned size;
		double res;
		if (line.compare(0, AbstractOccupancyGrid3D::binaryFileHeader.length(), AbstractOccupancyGrid3D::binaryFileHeader) == 0){
			std::string id;
			if (!AbstractGrid3D::readHeader(s, id, size, res))
				return false;

			GRIDMAP3D_DEBUG_STR("Reading binary grid3D type " << id);
		}

		// stream is now at binary data!
		this->clear();
		this->setResolution(res);

		if (size > 0)
			this->readBinaryData(s);

		if (size != this->size()){
			GRIDMAP3D_ERROR("Grid size mismatch: # read nodes (%zu) != # expected nodes (%d)\n", this->size(), size);
			return false;
		}

		return true;
	}

	const std::string AbstractOccupancyGrid3D::binaryFileHeader = "# GridMap3D Grid3D binary file";
}
