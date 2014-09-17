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

#include <hpp/core/path-logger.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/spline-path.hh>
#include <roboptim/core/function.hh>

namespace hpp{
	namespace core{
		
		
		PathLogger::PathLogger(const std::string& fileName){		
			file_.open( fileName.c_str(), std::ios::trunc  );//std::ios::app
			file_.seekp( std::ios::beg );

			if (!file_.good()) return;
			file_.flush();
		}
		
		

		void PathLogger::write(const std::string& msg){
			file_ << msg;
    		file_.flush();
		}
		
		void PathLogger::write(double nb){
			file_ << nb;
    		file_.flush();
		}
		
		void PathLogger::writeIterationCost(const double it, const double cost){
			file_ << it << "\t" << cost <<std::endl;
		}
		
		void PathLogger::generalInfos(const PathVectorPtr_t& pv,const PathPtr_t& sp){
			
		}
		
		
		void PathLogger::printPath(const PathVectorPtr_t& pv){
			file_ << *pv;
		}
		
		void PathLogger::printPath_xy(const PathVectorPtr_t& pv){
			// No need for matlab plotting, linear interpolation between singular points xy
		}
		
		
		// Singular points of pathVector of straightpaths
		
		void PathLogger::printSingularPoints(const PathVectorPtr_t& pv){

				// get the dimension
				size_t dim = pv->outputSize();

				// get the number of singular points
				size_t nbSp = pv->paths().size()+1;

				size_t i =0;
				Configuration_t current = (*pv)(pv->paths()[i]->timeRange().first);
				do {
					for (size_t j = 0; j < dim; ++j){
						file_ <<  current[j] << "\t" ;
					}
					file_ << std::endl;
					current= (*pv)(pv->paths()[i]->timeRange().second);
					i++;

				} while (i < nbSp-1);
				
				for (size_t j = 0; j < 2; ++j){
						file_ <<  current[j] << "\t" ;
				}
				file_ << std::endl;
		}
		
		// Singular points of pathVector of straightpaths ----------- XY
		
		void PathLogger::printSingularPoints_xy(const PathVectorPtr_t& pv){

				// get the dimension
				size_t dim = pv->outputSize();

				// get the number of singular points
				size_t nbSp = pv->paths().size()+1;

				size_t i =0;
				Configuration_t current = (*pv)(pv->paths()[i]->timeRange().first);
				do {
					for (size_t j = 0; j < 2; ++j){
						file_ <<  current[j] << "\t" ;
					}
					file_ << std::endl;
					current= (*pv->paths()[i])(pv->paths()[i]->timeRange().second);
					i++;

				} while (i < nbSp-1);
				
				for (size_t j = 0; j < 2; ++j){
						file_ <<  current[j] << "\t" ;
				}
				file_ << std::endl;
				
//				file_ << "-------------------------------------------------\t\n" << std::endl;
//				file_ << (*pv);
		}
		
		
		
		
		void PathLogger::printPath(const PathPtr_t& path){
			using boost::format;
			assert (path->outputSize() >= 2);
			roboptim::Function::value_type min = roboptim::Function::getLowerBound (path->timeRange ());
			roboptim::Function::value_type max = roboptim::Function::getUpperBound (path->timeRange ());
			roboptim::Function::value_type step = 0.01;
			
			if (min + step > max) throw std::string ("bad interval");
			SplinePathPtr_t sp = boost::dynamic_pointer_cast<SplinePath> (path);
			 
			for (double i = 0; i < 1. - step; i += step){ //i=step
				roboptim::Function::vector_t res = (*sp->spline()) (i * roboptim::tMax);
				for (size_t k =0; k < sp->spline()->outputSize();++k){
					file_ <<  res[k] <<  "\t" ;
				}
				file_ << std::endl;
			}
		}
		
		
		void PathLogger::printPath_xy(const PathPtr_t& path){
			using boost::format;
			assert (path->outputSize() >= 2);
			roboptim::Function::value_type min = roboptim::Function::getLowerBound (path->timeRange ());
			roboptim::Function::value_type max = roboptim::Function::getUpperBound (path->timeRange ());
			roboptim::Function::value_type step = 0.01;
			
			if (min + step > max) throw std::string ("bad interval");
			SplinePathPtr_t sp = boost::dynamic_pointer_cast<SplinePath> (path);
			 
			for (double i = step; i < 1. - step; i += step){
				double time = i;
				roboptim::Function::vector_t res = (*sp->spline()) (i * roboptim::tMax);
				file_ <<  time << "\t" <<res[0] <<  "\t" << res[1] <<std::endl;
			}
			
		}
		
		void PathLogger::printPath_xy(const CubicBSpline_t& spline){
			using boost::format;
			assert (spline.outputSize() >= 2);
			roboptim::Function::value_type min = roboptim::Function::getLowerBound (spline.timeRange ());
			roboptim::Function::value_type max = roboptim::Function::getUpperBound (spline.timeRange ());
			roboptim::Function::value_type step = 0.01;
			
			if (min + step > max) throw std::string ("bad interval");
			 
			for (double i = step; i < 1. - step; i += step){
				double time = i;
				roboptim::Function::vector_t res = spline(i * roboptim::tMax);
				file_ <<  time << "\t" <<res[0] <<  "\t" << res[1] <<std::endl;
			}
			
		}
		
		
		
		
		void PathLogger::printControlPoints(const PathPtr_t& path){

				
				SplinePathPtr_t sp = boost::dynamic_pointer_cast<SplinePath> (path);
				// get the dimension
				size_t splineDim = sp->spline()->outputSize();

				// get the number of control points
				size_t nbCp = sp->spline()->getNumberControlPoints();
				
				for(size_t i=0;i< nbCp;++i){
					for(size_t k =0; k < splineDim;++k){
						file_ << sp->spline()->parameters()[i*splineDim+k] << "\t" ;
					}
					file_ << std::endl;
				}

		}
		
		void PathLogger::printControlPoints_xy(const PathPtr_t& path){


				
				SplinePathPtr_t sp = boost::dynamic_pointer_cast<SplinePath> (path);
				// get the dimension
				size_t splineDim = sp->spline()->outputSize();

				// get the number of control points
				size_t nbCp = sp->spline()->getNumberControlPoints();
				
				for(size_t i=0;i< nbCp;++i){
					for(size_t k =0; k < 2;++k){
						file_ << sp->spline()->parameters()[i*splineDim+k] << "\t" ;
					}
					file_ << std::endl;
				}

		}
		
		
		void PathLogger::printControlPoints_xy(const CubicBSpline_t& spline){


				// get the dimension
				size_t splineDim = spline.outputSize();

				// get the number of control points
				size_t nbCp = spline.getNumberControlPoints();
				
				for(size_t i=0;i< nbCp;++i){
					for(size_t k =0; k < 2;++k){
						file_ << spline.parameters()[i*splineDim+k] << "\t" ;
					}
					file_ << std::endl;
				}

		}
	}
		
}
