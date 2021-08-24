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
 * CityBufferedFileWriter.h
 *
 * $Revision: 425 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_BUFFEREDFILEWRITER_H
#define CITY_BUFFEREDFILEWRITER_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

namespace dtsim {

/*--------------------------------------------------------*/
/*                      NOTICE !!!                        */
/*--------------------------------------------------------*/
/* This class must be updated to enhance the performance. */
/* Now, I leave only the skeleton for the class.          */
/*--------------------------------------------------------*/


#define BFW_APPEND(A) { append(); *buf << A; return *buf; }

/**
 * \class CityBufferedFileWriter
 *
 * \brief
 * Write test data to file
 *
 * Write user data to the text file with buffered method.
 */

class CityBufferedFileWriter {
public:
	/// output file handle
	std::ofstream *out;

	/// string buffer to keep the data temporarily
	std::stringstream *buf;

	/// block size limit
	int blimit;

public:
	/// Creates a CityBufferedFileWriter
	CityBufferedFileWriter() {
		out = 0;
	}

	/**
	 * Creates a CityBufferedFileWriter
	 *
	 * @param fileName string type filename 
	 */
	CityBufferedFileWriter(std::string fileName) {
		init(fileName, 1024);
	}

	/**
	 * Creates a CityBufferedFileWriter
	 *
	 * @param fileName string type filename
	 * @param _blimit block size limit
	 */
	CityBufferedFileWriter(std::string fileName, int _blimit) {
		init(fileName, _blimit);
	}

	/// Destroy a CityBufferedFileWriter
	~CityBufferedFileWriter() {
		if(out)
			close();
	}

	/// Initialize CityBufferedFileWriter
	void init(std::string fileName) {
		init(fileName, 1024);
	}

	/// Initialize CityBufferedFileWriter
	void init(std::string fileName, int _blimit) {
		out = new std::ofstream(fileName.c_str(), std::ios_base::app);
		buf = new std::stringstream();
		buf->str().resize(_blimit * 2);
		blimit = _blimit;
	}

	/// Append data to the string buffer & flush if the string buffer is full
	void append() {
		if(out == 0) {
			throw CityException(__func__, "outFile is not initialized");
		}

		if(buf->str().size() > blimit) {
			*out << buf->str();
			buf->str("");
		}
	}

	/// Flush CityBufferedFileWriter to the file
	void flush() {
		*out << buf->str();
		buf->str("");
	}

	/// Flush CityBufferedFileWriter to the file
	void close() {
		if(buf->str().size() > 0) {
			*out << buf->str();
		}
		out->close();

		delete out;
		delete buf;

		out = 0;
		buf = 0;
	}

	/// Operator 
	std::stringstream& operator<<(const std::string &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const char *val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const bool &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const short &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const unsigned short &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const int &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const unsigned int &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const long &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const unsigned long &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const float &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const double &val) { BFW_APPEND(val); }
	std::stringstream& operator<<(const long double &val) { BFW_APPEND(val); }
};


} /* namespace dtsim */

#endif /* CITY_BUFFEREDFILEWRITER_H */
