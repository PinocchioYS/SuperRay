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

#ifndef GRIDMAP3D_ABSTRACT_GRID3D_H
#define GRIDMAP3D_ABSTRACT_GRID3D_H

#include <cstddef>
#include <fstream>
#include <string>
#include <iostream>
#include <map>

namespace gridmap3D {

	/**
	 * This abstract class is an interface to all gridmap3Ds and provides a
	 * factory design pattern for readin and writing all kinds of Grid3Ds
	 * to files (see read()).
	 */
	class AbstractGrid3D {
		friend class StaticMapInit;
	public:
		AbstractGrid3D();
		virtual ~AbstractGrid3D() {};

		/// virtual constructor: creates a new object of same type
		virtual AbstractGrid3D* create() const = 0;

		/// returns actual class name as string for identification
		virtual std::string getGridType() const = 0;

		virtual double getResolution() const = 0;
		virtual void setResolution(double res) = 0;
		virtual size_t size() const = 0;
		virtual size_t memoryUsage() const = 0;
		virtual size_t memoryUsageNode() const = 0;
		virtual void getMetricMin(double& x, double& y, double& z) = 0;
		virtual void getMetricMin(double& x, double& y, double& z) const = 0;
		virtual void getMetricMax(double& x, double& y, double& z) = 0;
		virtual void getMetricMax(double& x, double& y, double& z) const = 0;
		virtual void getMetricSize(double& x, double& y, double& z) = 0;

		virtual void clear() = 0;

		//-- Iterator grid access

		// default iterator is leaf_iterator
//		class iterator_base;

		/// Write file header and complete grid to file (serialization)
		bool write(const std::string& filename) const;
		/// Write file header and complete grid to stream (serialization)
		bool write(std::ostream& s) const;

		/**
		 * Creates a certain Grid3D (factory pattern)
		 *
		 * @param id unique ID of Grid3D
		 * @param res resolution of Grid3D
		 * @return pointer to newly created Grid3D (empty). NULL if the ID is unknown!
		 */
		static AbstractGrid3D* createGrid(const std::string id, double res);

		/**
		 * Read the file header, create the appropriate class and deserialize.
		 * This creates a new grid3D which you need to delete yourself. If you
		 * expect or requre a specific kind of grid3D, use dynamic_cast afterwards
		 */
		static AbstractGrid3D* read(const std::string& filename);

		/// Read the file header, create the appropriate class and deserialize.
		/// This creates a new grid3D which you need to delete yourself.
		static AbstractGrid3D* read(std::istream &s);

		/**
		 * Read all nodes from the input stream (without file header),
		 * for this the grid needs to be already created.
		 * For general file IO, you
		 * should probably use AbstractGrid3D::read() instead.
		 */
		virtual std::istream& readData(std::istream &s) = 0;

		/// Write complete state of grid to stream (without file header) unmodified.
		/// Pruning the grid first produces smaller files (lossless compression)
		virtual std::ostream& writeData(std::ostream &s) const = 0;
	private:
		/// create private store, Construct on first use
		static std::map<std::string, AbstractGrid3D*>& classIDMapping();

	protected:
		static bool readHeader(std::istream &s, std::string& id, unsigned& size, double& res);
		static void registerGridType(AbstractGrid3D* grid);

		static const std::string fileHeader;
	};
} // end namespace


#endif
