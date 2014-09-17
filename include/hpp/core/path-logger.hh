//
// Copyright (c) 2014 CNRS
// Authors: Florian Valenza
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PATH_LOGGER_HH
# define HPP_CORE_PATH_LOGGER_HH

#include <string>
#include <fstream>
#include "logger.hh"
# include <roboptim/trajectory/sys.hh>
# include <hpp/core/fwd.hh>

namespace hpp{
	namespace core{
class PathLogger : public Logger{
public:
  PathLogger(const std::string& fileName);

  
		virtual ~PathLogger(){
			if (!file_.good()) return;
			file_.close();
		}
		
	
	virtual void write(const std::string& msg);
	
	virtual void write(double nb);
	
	void writeIterationCost(const double it, const double cost);
	
	void generalInfos(const PathVectorPtr_t& pv,const PathPtr_t& sp);
	
	void printPath(const PathVectorPtr_t& path);	
	
	void printPath(const PathPtr_t& path);	

	void printPath_xy(const PathVectorPtr_t& path);	
	
	void printPath_xy(const PathPtr_t& path);	
	
	void printPath_xy(const CubicBSpline_t& spline);
		
	void printSingularPoints(const PathVectorPtr_t& pv);
	
	void printSingularPoints_xy(const PathVectorPtr_t& pv);
	
	void printControlPoints(const PathPtr_t& path);
	
	void printControlPoints_xy(const PathPtr_t& path);
	
	void printControlPoints_xy(const CubicBSpline_t& spline);

private:
  std::ofstream file_;
}; // class PAthLogger
	} //  namespace core
}// namespace hpp

#endif // HPP_CORE_PATH_LOGGER_HH
