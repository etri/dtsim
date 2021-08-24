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
 * CityShapefileReader.cpp
 *
 * $Revision: 973 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <cstring>

#include "CityShapefileReader.h"
#include "CityGeometryFactory.h"

namespace dtsim {

CityShapefileReader::CityShapefileReader()
: shpHandle(nullptr), dbfHandle(nullptr), shapeCount(0), shapeType(0), fieldCount(0)
{}

CityShapefileReader::~CityShapefileReader()
{
	if (shpHandle != nullptr)
		SHPClose(shpHandle);

	if (dbfHandle != nullptr)
		DBFClose(dbfHandle);
}

bool CityShapefileReader::open(std::string fileName)
{
	if ((shpHandle != nullptr) || (dbfHandle != nullptr))
		return false;

	shpHandle = SHPOpen(fileName.c_str(), "rb");
	if (shpHandle == nullptr)
		return false;

	double minBound[4];
	double maxBound[4];
	SHPGetInfo(shpHandle, &shapeCount, &shapeType, minBound, maxBound);

	if ((shapeType != SHPT_POINT) && (shapeType != SHPT_ARC) && (shapeType != SHPT_POLYGON))
		return false;

	for (int i = 0; i < 4; i++) {
		shapeMinBound.push_back(minBound[i]);
		shapeMaxBound.push_back(maxBound[i]);
	}

	dbfHandle = DBFOpen(fileName.c_str(), "rb");
	if (dbfHandle == nullptr)
		return false;

	if (shapeCount != DBFGetRecordCount(dbfHandle))
		return false;

	fieldCount = DBFGetFieldCount(dbfHandle);
	if (fieldCount == 0)
		return false;

	char fieldName[12];
	DBFFieldType fieldType;
	int fieldWidth;
	int fieldDecimal;
	for (int i = 0; i < fieldCount; i++) {
		std::memset(fieldName, 0, sizeof fieldName);

		fieldType = DBFGetFieldInfo(dbfHandle, i, fieldName, &fieldWidth, &fieldDecimal);
		if (fieldType == FTInvalid)
			return false;

		fieldNames.push_back(fieldName);
		fieldTypes.push_back(fieldType);
		fieldWidths.push_back(fieldWidth);
		fieldDecimals.push_back(fieldDecimal);
	}

	return true;
}

void CityShapefileReader::close()
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

void CityShapefileReader::getShapeInfo(int& count, int& type, std::vector<double>& minBound, std::vector<double>& maxBound) const
{
	count = shapeCount;
	type = shapeType;
	minBound = shapeMinBound;
	maxBound = shapeMaxBound;
}

void CityShapefileReader::getFieldsInfo(int& count, std::vector<std::string>& names, std::vector<DBFFieldType>& types, std::vector<int>& widths, std::vector<int>& decimals) const
{
	count = fieldCount;
	names = fieldNames;
	types = fieldTypes;
	widths = fieldWidths;
	decimals = fieldDecimals;
}

int CityShapefileReader::readShape(int shapeIdx, int& nParts, std::vector<std::vector<double>>& xCoords, std::vector<std::vector<double>>& yCoords)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return SHPT_NULL;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return SHPT_NULL;

	SHPObject* shpObj = SHPReadObject(shpHandle, shapeIdx);
	if (shpObj == nullptr)
		return SHPT_NULL;

	std::vector<double> xVertices;
	std::vector<double> yVertices;

	if (shapeType == SHPT_POINT) {
		xVertices.push_back(shpObj->padfX[0]);
		yVertices.push_back(shpObj->padfY[0]);
		xCoords.push_back(xVertices);
		yCoords.push_back(yVertices);
		nParts = 1;
	} else if (shapeType == SHPT_ARC) {
		for (int i = 0; i < shpObj->nVertices; i++) {
			xVertices.push_back(shpObj->padfX[i]);
			yVertices.push_back(shpObj->padfY[i]);
		}
		xCoords.push_back(xVertices);
		yCoords.push_back(yVertices);
		nParts = 1;
	} else if (shapeType == SHPT_POLYGON) {
		if (shpObj->nParts > 1) {
			for (int i = 0, j = 1; i < shpObj->nParts; i++, j++) {
				int from, to;
				if (j == shpObj->nParts) {
					from = shpObj->panPartStart[i];
					to = shpObj->nVertices;
				} else {
					from = shpObj->panPartStart[i];
					to = shpObj->panPartStart[j];
				}

				for (int k = from; k < to; k++) {
					xVertices.push_back(shpObj->padfX[k]);
					yVertices.push_back(shpObj->padfY[k]);
				}
				xCoords.push_back(xVertices);
				yCoords.push_back(yVertices);

				xVertices.clear();
				yVertices.clear();
			}
			nParts = shpObj->nParts;
		} else {
			for (int i = 0; i < shpObj->nVertices; i++) {
				xVertices.push_back(shpObj->padfX[i]);
				yVertices.push_back(shpObj->padfY[i]);
			}
			xCoords.push_back(xVertices);
			yCoords.push_back(yVertices);
			nParts = 1;
		}
	} else {
		SHPDestroyObject(shpObj);
		return SHPT_NULL;
	}

	SHPDestroyObject(shpObj);
	return shapeType;
}

CityGeometry* CityShapefileReader::readShape(int shapeIdx)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return nullptr;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return nullptr;

	SHPObject* shpObj = SHPReadObject(shpHandle, shapeIdx);
	if (shpObj == nullptr)
		return nullptr;

	CityGeometry* geom;
	std::vector<double> coords;

	switch (shapeType) {
	case SHPT_POINT:
	case SHPT_POINTZ:
	case SHPT_POINTM:
		geom = CityGeometryFactory::instance()->createPoint(shpObj->padfX[0], shpObj->padfY[0]);
		break;

	case SHPT_ARC:
	case SHPT_ARCZ:
	case SHPT_ARCM:
		for (int i = 0; i < shpObj->nVertices; i++) {
			coords.push_back(shpObj->padfX[i]);
			coords.push_back(shpObj->padfY[i]);
		}
		geom = CityGeometryFactory::instance()->createArc(coords);
		break;

	case SHPT_POLYGON:
	case SHPT_POLYGONZ:
	case SHPT_POLYGONM:
		if (shpObj->nParts > 1) {
			std::vector<std::vector<double>> partCoords;

			for (int i = 0, j = 1; i < shpObj->nParts; i++, j++) {
				int from, to;
				if (j == shpObj->nParts) {
					from = shpObj->panPartStart[i];
					to = shpObj->nVertices;
				} else {
					from = shpObj->panPartStart[i];
					to = shpObj->panPartStart[j];
				}

				for (int k = from; k < to; k++) {
					coords.push_back(shpObj->padfX[k]);
					coords.push_back(shpObj->padfY[k]);
				}
				partCoords.push_back(coords);
				coords.clear();
			}
			geom = CityGeometryFactory::instance()->createPolygon(partCoords);
		} else {
			for (int i = 0; i < shpObj->nVertices; i++) {
				coords.push_back(shpObj->padfX[i]);
				coords.push_back(shpObj->padfY[i]);
			}
			geom = CityGeometryFactory::instance()->createPolygon(coords);
		}
		break;

	case SHPT_MULTIPOINT:
	case SHPT_MULTIPOINTZ:
	case SHPT_MULTIPOINTM:
	case SHPT_MULTIPATCH:
	case SHPT_NULL:
	default:
		SHPDestroyObject(shpObj);
		return nullptr;
	}

	SHPDestroyObject(shpObj);
	return geom;
}

bool CityShapefileReader::readShapeAttr(int shapeIdx, int fieldIdx, int& attr)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return false;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return false;

	if ((fieldIdx < 0) || (fieldIdx == fieldCount))
		return false;

	attr = DBFReadIntegerAttribute(dbfHandle, shapeIdx, fieldIdx);

	return true;
}

bool CityShapefileReader::readShapeAttr(int shapeIdx, int fieldIdx, double& attr)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return false;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return false;

	if ((fieldIdx < 0) || (fieldIdx == fieldCount))
		return false;

	attr = DBFReadDoubleAttribute(dbfHandle, shapeIdx, fieldIdx);

	return true;
}

bool CityShapefileReader::readShapeAttr(int shapeIdx, int fieldIdx, std::string& attr)
{
	if ((shpHandle == nullptr) || (dbfHandle == nullptr))
		return false;

	if ((shapeIdx < 0) || (shapeIdx == shapeCount))
		return false;

	if ((fieldIdx < 0) || (fieldIdx == fieldCount))
		return false;

	attr = DBFReadStringAttribute(dbfHandle, shapeIdx, fieldIdx);

	return true;
}

} /* namespace dtsim */
