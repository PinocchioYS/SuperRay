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


#include <quadmap/AbstractOccupancyQuadTree.h>
#include <quadmap/quadmap_types.h>

namespace quadmap {
	AbstractOccupancyQuadTree::AbstractOccupancyQuadTree(){
		// some sane default values:
		setOccupancyThres(0.5);   // = 0.0 in logodds
		setProbHit(0.7);          // = 0.85 in logodds
		setProbMiss(0.4);         // = -0.4 in logodds

		setClampingThresMin(0.1192); // = -2 in log odds
		setClampingThresMax(0.971); // = 3.5 in log odds
	}

	bool AbstractOccupancyQuadTree::writeBinary(const std::string& filename){
		std::ofstream binary_outfile(filename.c_str(), std::ios_base::binary);

		if (!binary_outfile.is_open()){
			QUADMAP_ERROR_STR("Filestream to " << filename << " not open, nothing written.");
			return false;
		}
		return writeBinary(binary_outfile);
	}

	bool AbstractOccupancyQuadTree::writeBinaryConst(const std::string& filename) const{
		std::ofstream binary_outfile(filename.c_str(), std::ios_base::binary);

		if (!binary_outfile.is_open()){
			QUADMAP_ERROR_STR("Filestream to " << filename << " not open, nothing written.");
			return false;
		}
		writeBinaryConst(binary_outfile);
		binary_outfile.close();
		return true;
	}

	bool AbstractOccupancyQuadTree::writeBinary(std::ostream &s){
		// convert to max likelihood first, this makes efficient pruning on binary data possible
		this->toMaxLikelihood();
		this->prune();
		return writeBinaryConst(s);
	}

	bool AbstractOccupancyQuadTree::writeBinaryConst(std::ostream &s) const{
		// write new header first:
		s << binaryFileHeader << "\n# (feel free to add / change comments, but leave the first line as it is!)\n#\n";
		s << "id " << this->getTreeType() << std::endl;
		s << "size " << this->size() << std::endl;
		s << "res " << this->getResolution() << std::endl;
		s << "data" << std::endl;

		writeBinaryData(s);

		if (s.good()){
			QUADMAP_DEBUG(" done.\n");
			return true;
		}
		else {
			QUADMAP_WARNING_STR("Output stream not \"good\" after writing tree");
			return false;
		}
	}

	bool AbstractOccupancyQuadTree::readBinaryLegacyHeader(std::istream &s, unsigned int& size, double& res) {

		if (!s.good()){
			QUADMAP_WARNING_STR("Input filestream not \"good\" in QuadTree::readBinary");
		}

		int tree_type = -1;
		s.read((char*)&tree_type, sizeof(tree_type));
		if (tree_type == 3){

			this->clear();

			s.read((char*)&res, sizeof(res));
			if (res <= 0.0){
				QUADMAP_ERROR("Invalid tree resolution: %f", res);
				return false;
			}

			s.read((char*)&size, sizeof(size));

			return true;
		}
		else {
			QUADMAP_ERROR_STR("Binary file does not contain a QuadTree!");
			return false;
		}
	}

	bool AbstractOccupancyQuadTree::readBinary(const std::string& filename){
		std::ifstream binary_infile(filename.c_str(), std::ios_base::binary);
		if (!binary_infile.is_open()){
			QUADMAP_ERROR_STR("Filestream to " << filename << " not open, nothing read.");
			return false;
		}
		return readBinary(binary_infile);
	}

	bool AbstractOccupancyQuadTree::readBinary(std::istream &s) {

		if (!s.good()){
			QUADMAP_WARNING_STR("Input filestream not \"good\" in QuadTree::readBinary");
		}

		// check if first line valid:
		std::string line;
		std::istream::pos_type streampos = s.tellg();
		std::getline(s, line);
		unsigned size;
		double res;
		if (line.compare(0, AbstractOccupancyQuadTree::binaryFileHeader.length(), AbstractOccupancyQuadTree::binaryFileHeader) == 0){
			std::string id;
			if (!AbstractQuadTree::readHeader(s, id, size, res))
				return false;

			QUADMAP_DEBUG_STR("Reading binary quadtree type " << id);
		}
		else{ // try to read old binary format:
			s.clear(); // clear eofbit of istream
			s.seekg(streampos);
			if (readBinaryLegacyHeader(s, size, res)){
				QUADMAP_WARNING_STR("You are using an outdated binary tree file format.");
			}
			else {
				QUADMAP_ERROR_STR("First line of QuadTree file header does not start with \"" << AbstractOccupancyQuadTree::binaryFileHeader << "\"");
				return false;
			}
		}
		// otherwise: values are valid, stream is now at binary data!
		this->clear();
		this->setResolution(res);

		if (size > 0)
			this->readBinaryData(s);

		if (size != this->size()){
			QUADMAP_ERROR("Tree size mismatch: # read nodes (%zu) != # expected nodes (%d)\n", this->size(), size);
			return false;
		}

		return true;
	}

	const std::string AbstractOccupancyQuadTree::binaryFileHeader = "# Quadmap QuadTree binary file";
}
