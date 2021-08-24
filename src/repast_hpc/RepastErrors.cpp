/*
 *   Repast for High Performance Computing (Repast HPC)
 *
 *   Copyright (c) 2010 Argonne National Laboratory
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with
 *   or without modification, are permitted provided that the following
 *   conditions are met:
 *
 *     Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *     Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *     Neither the name of the Argonne National Laboratory nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *   PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  RepastErrors.cpp
 *
 *  Created on: Feb 21, 2012
 *      Author: JTM
 */

#include "RepastProcess.h"
#include "RepastErrors.h"

namespace repast {

std::string err_msg(int idNum, std::string thrown_by, std::string reason, std::string explanation, std::string cause, std::string resolution){
  std::stringstream ss;
  ss << "REPAST_ERROR_" << idNum << "\n"
     << "   Thrown By:     " << thrown_by << "\n"
     << "   On Rank:       " << repast::RepastProcess::instance()->rank() << "\n"
     << "   Reason:        " << reason << "\n"
     << "   Explanation:   " << explanation << "\n"
     << "   Known Causes:  " << cause << "\n"
     << "   Resolution:    " << resolution << "\n";
  return ss.str();
}

std::string err_msg_omit_rank(int idNum, std::string thrown_by, std::string reason, std::string explanation, std::string cause, std::string resolution){
  std::stringstream ss;
  ss << "REPAST_ERROR_" << idNum << "\n"
     << "   Thrown By:     " << thrown_by << "\n"
     << "   Reason:        " << reason << "\n"
     << "   Explanation:   " << explanation << "\n"
     << "   Known Causes:  " << cause << "\n"
     << "   Resolution:    " << resolution << "\n";
  return ss.str();
}





}
