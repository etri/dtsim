/*                                                                                                                  
 *                                                                                                                   * Note: This license has also been called the "New BSD License" 
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
 * CityShapefileWriter.cpp
 *
 * $Revision$
 * $LastChangedDate$
 */

#include "CityShapefileWriter.h"

namespace dtsim {

CityShapefileWriter::CityShapefileWriter()
: shpHandle(nullptr), dbfHandle(nullptr), shapeCount(0), shapeType(0), fieldCount(0)
{}

CityShapefileWriter::~CityShapefileWriter()
{
	if (shpHandle != nullptr)
		SHPClose(shpHandle);

	if (dbfHandle != nullptr)
		DBFClose(dbfHandle);
}

bool CityShapefileWriter::create(std::string fileName, int type)
{
	shpHandle = SHPOpen(fileName.c_str(), "rb");
	if (shpHandle != nullptr)
		return false;

	shapeType = type;

	shpHandle = SHPCreate(fileName.c_str(), shapeType);
	if (shpHandle != nullptr)
		return false;

	dbfHandle = DBFOpen(fileName.c_str(), "rb");
	if (dbfHandle != nullptr)
		return false;

	dbfHandle = DBFCreate(fileName.c_str());
	if (dbfHandle != nullptr)
		return false;

	return true;
}

void CityShapefileWriter::close()
{
	if (shpHandle != nullptr) {
		SHPClose(shpHandle);
		shpHandle = nullptr;
	}

	if (dbfHandle != nullptr) {
		DBFClose(dbfHandle);
		dbfHandle = nullptr;
	}
}

void CityShapefileWriter::getShapeInfo(int& count, int& type, std::vector<double>& shapeMinBound, std::vector<double>& shapeMaxBound) const
{
	count = shapeCount;
	type = shapeType;

	double minBound[4];
	double maxBound[4];
	SHPGetInfo(shpHandle, nullptr, nullptr, minBound, maxBound);
	for (int i = 0; i < 4; i++) {
		shapeMinBound.push_back(minBound[i]);
		shapeMaxBound.push_back(maxBound[i]);
	}
}

void CityShapefileWriter::getFieldsInfo(int& count, std::vector<std::string>& names, std::vector<int>& types, std::vector<int>& widths, std::vector<int>& decimals) const
{
	count = fieldCount;
	names = fieldNames;
	types = fieldTypes;
	widths = fieldWidths;
	decimals = fieldDecimals;
}

bool CityShapefileWriter::writeShape(int shapeIdx, std::vector<double> xCoords, std::vector<double> yCoords)
{
	int nVertices = xCoords.size();
	double xArray[nVertices];
	double yArray[nVertices];

	SHPObject* shapeObject = SHPCreateObject(shapeType, shapeIdx, 1, nullptr, nullptr, nVertices, xArray, yArray, nullptr, nullptr);
	if (shapeObject == nullptr)
		return false;

	if (SHPWriteObject(shpHandle, shapeIdx, shapeObject) < 0) {
		SHPDestroyObject(shapeObject);
		return false;
	}

	SHPDestroyObject(shapeObject);

	shapeCount++;

	return true;
}

bool CityShapefileWriter::writeShape(int shapeIdx, CityGeometry* geometry)
{
	CityCoordinateSequence coordSeq;

	geometry->getCoordinates(coordSeq);

	int nVertices = coordSeq.getSize();
	double xArray[nVertices];
	double yArray[nVertices];

	SHPObject* shapeObject = SHPCreateObject(shapeType, shapeIdx, 1, nullptr, nullptr, nVertices, xArray, yArray, nullptr, nullptr);
	if (shapeObject == nullptr)
		return false;

	if (SHPWriteObject(shpHandle, shapeIdx, shapeObject) < 0) {
		SHPDestroyObject(shapeObject);
		return false;
	}

	SHPDestroyObject(shapeObject);

	shapeCount++;

	return true;
}

bool CityShapefileWriter::addField(std::string fieldName, int fieldType, int width, int decimals)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return false;

	DBFFieldType type;
	switch (fieldType) {
	case 0:
		type = FTString;
		break;
	case 1:
		type = FTInteger;
		break;
	case 2:
		type = FTDouble;
		break;
	case 3:
		type = FTLogical;
		break;
	case 4:
		type = FTDate;
		break;
	default:
		return false;
	}

	if (DBFAddField(dbfHandle, fieldName.c_str(), type, width, decimals) == -1)
		return false;

	fieldCount++;

	fieldNames.push_back(fieldName);
	fieldTypes.push_back(fieldType);
	fieldWidths.push_back(width);
	fieldDecimals.push_back(decimals);

	return true;
}

bool CityShapefileWriter::writeShapeAttr(int shapeIdx, int fieldIdx, int attr)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return false;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return false;

	if ((fieldIdx < 0) || (fieldIdx == fieldCount))
		return false;

	if (DBFWriteIntegerAttribute(dbfHandle, shapeIdx, fieldIdx, attr) == 0)
		return false;

	return true;
}

bool CityShapefileWriter::writeShapeAttr(int shapeIdx, int fieldIdx, double attr)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return false;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return false;

	if ((fieldIdx < 0) || (fieldIdx == fieldCount))
		return false;

	if (DBFWriteDoubleAttribute(dbfHandle, shapeIdx, fieldIdx, attr) == 0)
		return false;

	return true;
}

bool CityShapefileWriter::writeShapeAttr(int shapeIdx, int fieldIdx, std::string attr)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return false;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return false;

	if ((fieldIdx < 0) || (fieldIdx == fieldCount))
		return false;

	if (DBFWriteStringAttribute(dbfHandle, shapeIdx, fieldIdx, attr.c_str()) == 0)
		return false;

	return true;
}

} /* namespace dtsim */
