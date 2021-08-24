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
 * CityCSVFileWriter.cpp
 *
 * $Revision: 625 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include <sstream>
#include "CityCSVFileWriter.h"

namespace dtsim {

CityCSVFileWriter::CityCSVFileWriter(std::string filePath_, char delimiter_)
: filePath(filePath_), delimiter(delimiter_)
{
	fs.open(filePath, std::ios::out | std::ios::app);
}

CityCSVFileWriter::~CityCSVFileWriter()
{
	if (fs.is_open())
		fs.close();
}

bool CityCSVFileWriter::open(std::string filePath_, char delimiter_)
{
	if (fs.is_open())
		return false;

	filePath = filePath_;
	delimiter = delimiter_;

	fs.open(filePath, std::ios::out | std::ios::app);

	if (!fs.is_open())
		return false;

	return true;
}

bool CityCSVFileWriter::write(std::vector<std::string>& line)
{
	if (!fs.is_open())
		return false;

	std::string output;

	std::size_t n = line.size() - 1;
	for (std::size_t i = 0; i < n; i++)
		output.append(line[i]).append(",");
	output.append(line[n]);

	fs << output << std::endl;
	return true;
}

void CityCSVFileWriter::close()
{
	if (fs.is_open())
		fs.close();
}

} /* namespace dtsim */
