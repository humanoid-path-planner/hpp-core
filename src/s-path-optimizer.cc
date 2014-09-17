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

#include <limits>
#include <deque>
#include <cstdlib>
#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/spline-path.hh>
#include <hpp/core/s-path-optimizer.hh>
#include <hpp/core/problem.hh>
#include <roboptim/trajectory/cubic-b-spline.hh>
#include <hpp/core/spline-cost.hh>
#include <roboptim/core/solver-factory.hh>
#include <hpp/core/path-logger.hh>


#include <roboptim/trajectory/visualization/trajectory.hh>



typedef roboptim::Solver
      <roboptim::DifferentiableFunction,
       boost::mpl::vector<roboptim::LinearFunction,
                          roboptim::DifferentiableFunction> >
      opt_solver_t;
      
using namespace roboptim;

namespace hpp {
	namespace core {
		
		
		CubicBSpline_t SPathOptimizer::pvToCompleteSpline(const PathVectorPtr_t& pv) const{
				
			// get the time range
			double t0 = pv->timeRange().first; 
			double tf = pv->timeRange().second;
			interval_t tr = roboptim::CubicBSpline::makeInterval(t0,tf);

			// get the dimension
			size_t dim = pv->outputSize();

			// get the number of control points
			size_t nbCp = pv->paths().size()+5; //multiplicity of 3 at start and end

			// create temporary param vector (associated to PathVector)
			roboptim::CubicBSpline::vector_t tmpParams ( (nbCp) * dim);
			tmpParams.setConstant (sqrt (-1));
			size_t i =0;
			
			Configuration_t current = (*pv)(pv->paths()[i]->timeRange().first);
			do {
				
			  for (size_t j = 0; j < dim; ++j) {
				tmpParams[dim*i + j] = current[j];
			  }
			  
			  if ( i > 1 && i < nbCp-3) {
				current= (*pv->paths()[i-2])(pv->paths()[i-2]->timeRange().second);
			  }			  
			  i++;
			  
			} while (i < nbCp);		
					
			// Creation of the spline
			return CubicBSpline (tr,dim,tmpParams,"cubic-b-spline");
		}
		
		CubicBSpline_t SPathOptimizer::pvToSmallSpline(const PathVectorPtr_t& pv) const{
				
			// get the time range
			double t0 = pv->timeRange().first; 
			double tf = pv->timeRange().second;
			interval_t tr = roboptim::CubicBSpline::makeInterval(t0,tf);

			// get the dimension
			size_t dim = pv->outputSize();

			// get the number of control points
			size_t nbCp = pv->paths().size()+5;

			// create temporary param vector (associatd to PathVector)
			roboptim::CubicBSpline::vector_t tmpParams ( (nbCp) * dim);
			tmpParams.setConstant (sqrt (-1));
			size_t i =0;
			Configuration_t current = (*pv)(pv->paths()[i]->timeRange().first);
			do {
				for (size_t j = 0; j < dim; ++j) {
					tmpParams[dim*i + j] = current[j];
				}				
				if ( i > 1 && i < nbCp -3) {
					current= (*pv->paths()[i-2])(pv->paths()[i-2]->timeRange().second);
				}
				i++;
			} while (i < nbCp);
			
			
			//last = (*pv)(pv->timeRange().second() ;
			
			i = 0;		
			// Create param vector of small spline
			size_t newDim = 2;
			size_t index=0;
			roboptim::CubicBSpline::vector_t params ( nbCp * newDim);		
			params.setConstant (sqrt (-1));	
			
			do{
				for(size_t k =0; k < newDim; ++k){
					if (k==0) index=0;
					else if (k==1) index=1;
					else if (k==2) index=5;		// index for Theta_z here (if spline.size() > 2)
					params[newDim*i + k] = tmpParams[dim*i+index];
				}
				i++;
			} while (i < nbCp);
					
			// Creation of the spline
			return CubicBSpline (tr,newDim,params,"cubic-b-spline");
		}
		
		
		PathVectorPtr_t SPathOptimizer::completeSplineToPv(const PathVectorPtr_t& original, const CubicBSpline_t& spline) const{
			
			// Copy the input spline
			CubicBSpline_t extendedSpline(spline);

			
			// Create SplinePath from extended spline 
			SplinePathPtr_t extendedSplinePath = SplinePath::create(extendedSpline);
			
			PathVectorPtr_t result;
			result = PathVector::create(original->outputSize ());
			result->appendPath (extendedSplinePath);
			return result;
			
		}
		
		PathVectorPtr_t SPathOptimizer::smallSplineToPv(const PathVectorPtr_t& original, const CubicBSpline_t& spline) const{	
			// Create extendedSpline
			// Get the dimensions
			size_t newDim = original->outputSize();
			size_t smallDim = spline.outputSize();

			// get the number of control points
			size_t nbCp = spline.getNumberControlPoints();
		
			Configuration_t originalConfig = (*original)(original->paths()[0]->timeRange().first);
			roboptim::CubicBSpline::vector_t newParams ( nbCp * newDim);
			newParams.setConstant (sqrt (-1));	
			
			// Filling the control point vector
			for(size_t i=0;i< nbCp;++i){
				for(size_t k =0; k < newDim;++k){
					switch(k){
						case 0:
							newParams[i*newDim + k ] = spline.parameters()[i*smallDim+0];
						break;
						case 1:
							newParams[i*newDim + k ] = spline.parameters()[i*smallDim+1];
						break;
						default:
							newParams[i*newDim + k ] = originalConfig[k];
						break;
					}
				}
				if ( i > 1 && i < nbCp -3) {
					originalConfig = (*original->paths()[i-2])(original->paths()[i-2]->timeRange().second);
				}
			}
			
						
			CubicBSpline_t extendedSpline(original->timeRange(),newDim,newParams,"cubic-B-Spline"); 
			// Create SplinePath from extended spline
			SplinePathPtr_t extendedSplinePath = SplinePath::create(extendedSpline);
			
			PathVectorPtr_t result;
			result = PathVector::create(original->outputSize ());
			result->appendPath (extendedSplinePath);
			return result;
			
		}

		SPathOptimizerPtr_t
		SPathOptimizer::create (const Problem& problem)
		{
			SPathOptimizer* ptr = new SPathOptimizer (problem);
			return SPathOptimizerPtr_t (ptr);
		}

		SPathOptimizer::SPathOptimizer (const Problem& problem) :
		PathOptimizer (problem)
		{
		}
		

	
		PathVectorPtr_t SPathOptimizer::optimize (const PathVectorPtr_t& path) const
		{

				PathVectorPtr_t resultPv;
				CubicBSpline_t splineTmp = hpp::core::SPathOptimizer::pvToSmallSpline(path);

				#ifdef basicCost
				hpp::core::SplineCost cost(splineTmp); // basic_cost
				#elif defined quadraticError
				hpp::core::SplineCost cost(splineTmp,path); // quadratic_error
				#elif defined obstacleAvoidance
				hpp::core::SplineCost cost(splineTmp,problem().robot(),path); // collision avoidance
				#endif

				// Creation of the optimization problem
				opt_solver_t::problem_t problem(cost);
				problem.startingPoint() = splineTmp.parameters();
				splineTmp.freezeCurveStart (problem);
				splineTmp.freezeCurveEnd (problem);
				
				SolverFactory<opt_solver_t> factory ("ipopt", problem);
				opt_solver_t& solver = factory ();
				
				// Ipopt-specific parameters
				// WARNING: these parameters may not be relevant! These are only set to
				// prevent hour-long unit testing...
				solver.parameters()["ipopt.linear_solver"].value = "mumps";//IPOPT_LINEAR_SOLVER;
				solver.parameters()["ipopt.max_iter"].value = 400;
				solver.parameters()["ipopt.tol"].value = 10e-5;
				solver.parameters()["ipopt.acceptable_tol"].value = 5e-4;
				solver.parameters()["ipopt.mu_strategy"].value = "adaptive";
				
				opt_solver_t::result_t res = solver.minimum();
								

				switch (res.which ())
				{
					case GenericSolver::SOLVER_VALUE:
					{
						Result& result_opt = boost::get<Result> (res);
						CubicBSpline optimizedSpline (splineTmp.timeRange(), splineTmp.outputSize(), result_opt.x, "after");
						resultPv = smallSplineToPv(path,optimizedSpline);
						break;
					}

					case GenericSolver::SOLVER_NO_SOLUTION:
					{
						std::cerr << "No solution" << std::endl;
						assert(false);
					}
					
					case GenericSolver::SOLVER_VALUE_WARNINGS:
					{
						ResultWithWarnings& result_opt = boost::get<ResultWithWarnings> (res);
						CubicBSpline optimizedSpline (splineTmp.timeRange(), splineTmp.outputSize(), result_opt.x, "after");
						resultPv = smallSplineToPv(path,optimizedSpline);
						break;
					}

					case GenericSolver::SOLVER_ERROR:
					{
						SolverError& result = boost::get<SolverError> (res);
						std::cerr << result << std::endl;
						assert(false);
					}

					default:
					{
						assert(false);
						break;
					}
				}
			
				return resultPv;	
		}

  } // namespace core
} // namespace hpp


