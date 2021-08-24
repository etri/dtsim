/*
 * Note: This license has also been called the "New BSD License" 
 * or "Modified BSD License".
 * 
 * Copyright (c) 2021 Electronics and Telecommunications Research 
 * Institute All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * CityShapefileWriter
 *
 * $Revision$
 * $LastChangedDate$
 */

#ifndef CITY_SHAPEFILE_WRITER_H
#define CITY_SHAPEFILE_WRITER_H

#include <string>
#include <vector>

#include "shapefil.h"
#include "CityGeometry.h"

namespace dtsim {

/**
 * \class CityShapefileWriter
 *
 * \brief
 * Shapefile writer.
 *
 * This uses the Shapefile C Library(http://shapelib.maptools.org).
 */
class CityShapefileWriter {
private:
	/// handle returned by SHPCreate()
	SHPHandle shpHandle;

	/// handle returned by DBFCreate()
	DBFHandle dbfHandle;

	int shapeCount;

	int shapeType;

	int fieldCount;

	std::vector<std::string> fieldNames;

	std::vector<int> fieldTypes;

	std::vector<int> fieldWidths;

	std::vector<int> fieldDecimals;

public:
	CityShapefileWriter();
	virtual ~CityShapefileWriter();

	bool create(std::string fileName, int type);

	void close();

	int getShapeType() const {
		return shapeType;
	}

	int getShapeCount() const {
		return shapeCount;
	}

	void getShapeInfo(int& count, int& type, std::vector<double>& shapeMinBound, std::vector<double>& shapeMaxBound) const;

	int getFieldCount() const {
		return fieldCount;
	}

	std::vector<std::string> getFieldNames() const {
		return fieldNames;
	}

	std::vector<int> getFieldTypes() const {
		return fieldTypes;
	}

	std::vector<int> getFieldWidths() const {
		return fieldWidths;
	}

	std::vector<int> getFieldDecimals() const {
		return fieldDecimals;
	}

	void getFieldsInfo(int& count, std::vector<std::string>& names, std::vector<int>& types, std::vector<int>& widths, std::vector<int>& decimals) const;

	bool writeShape(int shapeIdx, std::vector<double> xCoords, std::vector<double> yCoords);

	bool writeShape(int shapeIdx, CityGeometry* geometry);

	bool addField(std::string fieldName, int fieldType, int width, int decimals);

	bool writeShapeAttr(int shapeIdx, int fieldIdx, int attr);

	bool writeShapeAttr(int shapeIdx, int fieldIdx, double attr);

	bool writeShapeAttr(int shapeIdx, int fieldIdx, std::string attr);
};

} /* namespace dtsim */

#endif /* CITY_SHAPEFILE_WRITER_H */
