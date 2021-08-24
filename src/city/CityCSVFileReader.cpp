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
 * CityCSVFileReader.cpp
 *
 * $Revision: 625 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <sstream>
#include "CityCSVFileReader.h"

namespace dtsim {

CityCSVFileReader::CityCSVFileReader(std::string filePath_, char delimiter_, char quote_escape_)
: filePath(filePath_), delimiter(delimiter_), quote_escape(quote_escape_)
{
	fs.open(filePath, std::ios::in);
}

CityCSVFileReader::~CityCSVFileReader()
{
	if (fs.is_open())
		fs.close();
}

bool CityCSVFileReader::open(std::string filePath_, char delimiter_, char quote_escape_)
{
	if (fs.is_open())
		return false;

	filePath = filePath_;
	delimiter = delimiter_;
	quote_escape = quote_escape_;

	fs.open(filePath, std::ios::in);

	if (!fs.is_open())
		return false;

	return true;
}

bool CityCSVFileReader::read(std::vector<std::string>& row)
{
	if (!fs.is_open())
		return false;

	std::string line;

	std::getline(fs, line);
	if (!fs.good())
		return false;

	std::stringstream ss;
	bool quotes = false;

	for (int i = 0, n = line.length(); i < n; i++) {
		char c = line[i];

		if (!quotes && (c == delimiter)) {
			row.push_back(ss.str());
			ss.str("");
		} else if (!quotes && (c == quote_escape)) {
			quotes = true;
		} else if (quotes && (c == quote_escape)) {
			quotes = false;
		} else {
			ss << c;
		}
	}

	if (quotes)
		return false;

	if (!quotes)
		row.push_back(ss.str());

	return true;
}

void CityCSVFileReader::close()
{
	if (fs.is_open())
		fs.close();
}

} /* namespace dtsim */
