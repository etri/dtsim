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
 * CityShapefileReader
 *
 * $Revision: 973 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_SHAPEFILE_READER_H
#define CITY_SHAPEFILE_READER_H

#include <string>
#include <vector>

#include "shapefil.h"
#include "CityGeometry.h"

namespace dtsim {

/**
 * \class CityShapefileReader
 *
 * \brief
 * Shapefile reader.
 *
 * This uses the Shapefile C Library(http://shapelib.maptools.org).
 *
 * - Shape Types defined in shapefil.h
 *
 * // 2D Shape Type
 * #define SHPT_NULL         // 0
 * #define SHPT_POINT        // 1, points
 * #define SHPT_ARC          // 3, arcs (polylines, possible in parts)
 * #define SHPT_POLYGON      // 5, polygons (possible in parts)
 * #define SHPT_MULTIPOINT   // 8, multipoint (related points)
 *
 * // 3D Shape Type (may include "measure" values for vertices)
 * #define SHPT_POINTZ       // 11
 * #define SHPT_ARCZ         // 13
 * #define SHPT_POLYGONZ     // 15
 * #define SHPT_MULTIPOINTZ  // 18
 *
 * // 2D + Measure Types
 * #define SHPT_POINTM       // 21
 * #define SHPT_ARCM         // 23
 * #define SHPT_POLYGONM     // 25
 * #define SHPT_MULTIPOINTM  // 28
 *
 * // Complex (TIN-like) with Z, and Measure
 * #define SHPT_MULTIPATCH   // 31
 *
 *
 * - DBF Field Types defined in shapefil.h
 *
 * typedef enum {
 *    FTString   // 0, fiexd length string field
 *    FTInteger  // 1, numeric field with no decimals
 *    FTDouble   // 2, numeric field with decimals
 *    FTLogical  // 3, logical field
 *    FTDate     // 4, date field
 *    FTInvalid  // 5, not a recognised field type
 * } DBFFieldType;
 */

class CityShapefileReader {
private:
	/// handle returned by SHPOpen()
	SHPHandle shpHandle;

	/// handle returned by DBFOpen()
	DBFHandle dbfHandle;

	/// the number of entities/structures
	int shapeCount;

	/// shape type of this file.
	int shapeType;

	/// minimum bounds in x, y, z and m dimensions
	std::vector<double> shapeMinBound;

	/// maximum bounds in x, y, z and m dimensions
	std::vector<double> shapeMaxBound;

	/// the number of fields in DBF file
	int fieldCount;

	/// the name of each fields in DBF file
	std::vector<std::string> fieldNames;

	/// the type of each fields in DBF file
	std::vector<DBFFieldType> fieldTypes;

	/// the width of each fields in DBF file
	std::vector<int> fieldWidths;

	/// the number of decimal places precision defined for each fields in DBF file
	/// this is zero for integer fields, or non-numeric fields
	std::vector<int> fieldDecimals;

public:
	CityShapefileReader();
	virtual ~CityShapefileReader();

	/// open .shp and .dbf files
	bool open(std::string fileName);

	/// close the .shp and .dbf files
	void close();

	/// return the shape type of this shapefile.
	int getShapeType() const {
		return shapeType;
	}

	/// return the number of entities/structures
	int getShapeCount() const {
		return shapeCount;
	}

	/// return the shape information of this shapefile
	void getShapeInfo(int& count, int& type, std::vector<double>& minBound, std::vector<double>& maxBound) const;

	/// return the number of fields count
	int getFieldCount() const {
		return fieldCount;
	}

	/// return the field names
	std::vector<std::string> getFieldNames() const {
		return fieldNames;
	}

	/// return the field types
	std::vector<DBFFieldType> getFieldTypes() const {
		return fieldTypes;
	}

	/// return the field widths
	std::vector<int> getFieldWidths() const {
		return fieldWidths;
	}

	/// return the field decimals
	std::vector<int> getFieldDecimals() const {
		return fieldDecimals;
	}

	/// return the field information of this shapefile
	void getFieldsInfo(int& count, std::vector<std::string>& names, std::vector<DBFFieldType>& types, std::vector<int>& widths, std::vector<int>& decimals) const;

	/**
	 * read a single structure, or entity from shapefile.
	 *
	 * @param shapeIdx the entity number of the shape to read
	 * @param nParts number of parts of the shape
	 * @param xCoords vector of x-coordinate for each part
	 * @param yCoords vector of y-coordinate for each part
	 * @return return shapefile type
	 */
	int readShape(int shapeIdx, int& nParts, std::vector<std::vector<double>>& xCoords, std::vector<std::vector<double>>& yCoords);

	/**
	 * read a single structure, or entity from shapefile.
	 * creates CityGeometry object with shape type using CityGeometryFactory
	 *
	 * @param shapeIdx the entity number of the shape to read
	 * @return return CityGeometry object
	 */
	CityGeometry* readShape(int shapeIdx);

	/**
	 * read the value of one field and return it as an integer
	 *
	 * @param shapeIdx the record number(shape number) from which the field value should be read
	 * @param fieldIdx the field within the selected record that should be read
	 * @param attr returned integer
	 */
	bool readShapeAttr(int shapeIdx, int fieldIdx, int& attr);

	/**
	 * read the value of one field and return it as a double
	 *
	 * @param shapeIdx the record number(shape number) from which the field value should be read
	 * @param fieldIdx the field within the selected record that should be read
	 * @param attr returned double
	 */
	bool readShapeAttr(int shapeIdx, int fieldIdx, double& attr);

	/**
	 * read the value of one field and return it as a string
	 *
	 * @param shapeIdx the record number(shape number) from which the field value should be read
	 * @param fieldIdx the field within the selected record that should be read
	 * @param attr returned double
	 */
	bool readShapeAttr(int shapeIdx, int fieldIdx, std::string& attr);
};

} /* namespace dtsim */

#endif /* CITY_SHAPEFILE_READER_H */
