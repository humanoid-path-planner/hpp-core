// Copyright (c) 2015, 2016, 2017 LAAS-CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_SRC_IMPLICIT_FUNCTION_HH
# define HPP_CORE_SRC_IMPLICIT_FUNCTION_HH

#include <Eigen/Geometry>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/matrix-view.hh>
#include <hpp/constraints/tools.hh>
#include <hpp/core/comparison-type.hh>
#include <hpp/core/explicit-numerical-constraint.hh>

namespace hpp {
  namespace core {

    typedef hpp::pinocchio::liegroup::VectorSpaceOperation <3, false> R3;
    typedef hpp::pinocchio::liegroup::SpecialOrthogonalOperation <3> SO3;
    typedef hpp::pinocchio::liegroup::CartesianProductOperation <R3, SO3>
    R3xSO3;
    typedef se3::SpecialEuclideanOperation <3> SE3;
    typedef hpp::pinocchio::LiegroupType LiegroupType;

    struct JacobianVisitor : public boost::static_visitor <>
    {
      JacobianVisitor (vectorIn_t qOut, vectorIn_t f_qIn,
                       matrixIn_t Jf, const Eigen::MatrixBlocks <false, false>&
                       outJacobian, const Eigen::MatrixBlocks <false, false>&
                       inJacobian, matrixOut_t result) :
        qOut_ (qOut), f_qIn_ (f_qIn), Jf_ (Jf), outJacobian_ (outJacobian),
        inJacobian_ (inJacobian), result_ (result)
      {
      }

      template <typename LgT> void operator () (const LgT&);

      vectorIn_t qOut_, f_qIn_;
      matrixIn_t Jf_;
      const Eigen::MatrixBlocks <false, false>& outJacobian_;
      const Eigen::MatrixBlocks <false, false>& inJacobian_;
      matrixOut_t result_;
    }; // struct JacobianVisitor

    template <> inline void JacobianVisitor::operator () <R3xSO3 >
      (const R3xSO3&)
    {
      using Eigen::MatrixBlocks;
      using Eigen::BlockIndex;
      typedef hpp::constraints::BlockIndex BlockIndex;
      hppDout (info, "result_ = " << std::endl << result_);
      // Fill R^3 part
      assert (outJacobian_.nbRows () == 6);
      Eigen::MatrixBlocks <false, false> tmp (outJacobian_.block (0, 0, 3, 3));
      tmp.lview (result_) = matrix3_t::Identity ();
      hppDout (info, "result_ = " << std::endl << result_);
      // extract 3 top rows of inJacobian_
      segments_t cols (inJacobian_.cols ());
      MatrixBlocks <false, false> inJacobian
        (inJacobian_.block (0, 0, 3, BlockIndex::cardinal (cols)));
      inJacobian.lview (result_) = -Jf_.topRows <3> ();
      hppDout (info, "result_ = " << std::endl << result_);
      // Fill SO(3) part
      // extract 3 bottom rows of inJacobian_
      inJacobian = inJacobian_.block (3, 0, 3, BlockIndex::cardinal (cols));
      // extract 3x3 bottom left part of outJacobian_
      MatrixBlocks <false, false> outJacobian (outJacobian_.block (3, 3, 3, 3));
      assert (qOut_.size () == 7);
      assert (f_qIn_.size () == 7);
      matrix3_t R_out
        (Eigen::Quaterniond (qOut_.tail <4> ()).toRotationMatrix ());
      matrix3_t R_f
        (Eigen::Quaterniond (f_qIn_.tail <4> ()).toRotationMatrix ());
      // \f$R_f^T R_{out}\f$
      matrix3_t R_f_T_R_out (R_f.transpose () * R_out);
      matrix3_t Jlog_R_f_T_R_out;
      vector3_t r;
      value_type theta;
      constraints::logSO3 (R_f_T_R_out, theta, r);
      constraints::JlogSO3 (theta, r, Jlog_R_f_T_R_out);
      outJacobian.lview (result_) = Jlog_R_f_T_R_out;
      hppDout (info, "result_ = " << std::endl << result_);
      inJacobian.lview (result_) = -Jlog_R_f_T_R_out * R_out.transpose () *
        R_f * Jf_.bottomRows <3> ();
      hppDout (info, "result_ = " << std::endl << result_);
    }

    template <> inline void JacobianVisitor::operator () <SE3 > (const SE3&)
    {
      assert (outJacobian_.nbRows () == 6);
      // extract 3 top rows of inJacobian_
      assert (qOut_.size () == 7);
      assert (f_qIn_.size () == 7);
      matrix3_t Rout (Eigen::Quaterniond (qOut_.tail <4> ()).
                      toRotationMatrix ());
      vector3_t pOut (qOut_.head <3> ());
      Transform3f Mout (Rout, pOut);
      matrix3_t Rf (Eigen::Quaterniond (f_qIn_.tail <4> ()).
                    toRotationMatrix ());
      vector3_t pf (f_qIn_.head <3> ());
      Transform3f Mf (Rf, pf);
      // \f$Mf^{-1} M_{out}\f$
      Transform3f Mf_inverse_Mout (Mf.inverse () * Mout);
      matrix6_t Jlog_Mf_inverse_Mout;
      constraints::JlogSE3 (Mf_inverse_Mout, Jlog_Mf_inverse_Mout);
      outJacobian_.lview (result_) = Jlog_Mf_inverse_Mout;
      matrix_t inJ (6, Jf_.cols ());
      inJ.topRows <3> () =
        Rout.transpose () * se3::skew (pOut - pf) * Rf * Jf_.bottomRows <3> ()
        - Rout.transpose () * Rf * Jf_.topRows <3> ();
      inJ.bottomRows <3> () = -Rout.transpose () * Rf * Jf_.bottomRows <3> ();
      inJacobian_.lview (result_) = Jlog_Mf_inverse_Mout * inJ;
    }

    template <typename LgT> void JacobianVisitor::operator () (const LgT&)
    {
      for (size_type i=0; i<outJacobian_.nbRows (); ++i) {
        outJacobian_.lview (result_) (i, i) = 1;
      }
      inJacobian_.lview (result_) = -Jf_;
    }
      

    HPP_PREDEF_CLASS (ImplicitFunction);
    typedef boost::shared_ptr <ImplicitFunction> ImplicitFunctionPtr_t;

    /// Function of the form f (q) = q2 - g (q1)
    ///
    /// where
    ///  \li q2 is a vector composed of a subset of configuration variables of
    ///      q,
    ///  \li q1 is the vector composed of the other configuration variables of
    ///      q,
    ///  g is a differentiable function with values in  a Lie group.
    class ImplicitFunction : public DifferentiableFunction
    {
    public:
      static ImplicitFunctionPtr_t create
      (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
       const segments_t& inputConf, const segments_t& inputVelocity,
       const segments_t& outputConf, const segments_t& outputVelocity)
      {
	ImplicitFunction* ptr = new ImplicitFunction
	  (robot, function, inputConf, inputVelocity, outputConf,
           outputVelocity);
	return ImplicitFunctionPtr_t (ptr);
      }

      /// Get function that maps input variables to output variables
      const DifferentiableFunctionPtr_t& inputToOutput () const
      {
        return inputToOutput_;
      }

    protected:
      ImplicitFunction (const DevicePtr_t& robot,
			const DifferentiableFunctionPtr_t& function,
			const segments_t& inputConf,
			const segments_t& inputVelocity,
                        const segments_t& outputConf,
			const segments_t& outputVelocity)
	: DifferentiableFunction (robot->configSize (), robot->numberDof (),
				  LiegroupSpace::Rn
                                  (function->outputSpace ()->nv ())),
	  robot_ (robot), inputToOutput_ (function),
          inputConfIntervals_ (inputConf),
	  inputDerivIntervals_ (inputVelocity),
          outputConfIntervals_ (outputConf),
	  outputDerivIntervals_ (outputVelocity), outJacobian_ (),
          inJacobian_ (), f_qIn_ (function->outputSpace ()),
          qOut_ (function->outputSpace ()), result_ (outputSpace ())
      {
	// Check input consistency
	// Each configuration variable is either input or output
	assert (function->inputSize () + function->outputSize () <=
		robot->configSize ());
	// Each velocity variable is either input or output
	assert (function->inputDerivativeSize () +
		function->outputDerivativeSize () <= robot->numberDof ());
	qIn_.resize (function->inputSize ());
	Jf_.resize (function->outputDerivativeSize (),
                    function->inputDerivativeSize ());
	assert (BlockIndex::cardinal (outputConf) == function->outputSize ());
	// Sum of velocity output interval sizes equal function output
	// derivative size
	assert (BlockIndex::cardinal (outputVelocity) ==
                function->outputDerivativeSize ());
        computeJacobianBlocks ();
      }

      /// Compute q_{output} - f (q_{input})
      void impl_compute (LiegroupElement& result, vectorIn_t argument) const
      {
        using Eigen::MatrixBlocks;
        hppDout (info, "argument=" << argument.transpose ());
        // Store q_{output} in result
        qOut_.vector () = MatrixBlocks <false, true>
          (outputConfIntervals_).rview (argument);
        hppDout (info, "qOut_=" << qOut_);
        // fill in q_{input}
        qIn_ = MatrixBlocks <false, true>
          (inputConfIntervals_).rview (argument);
        hppDout (info, "qIn_=" << qIn_);
        // compute  f (q_{input}) -> output_
	inputToOutput_->value (f_qIn_, qIn_);
        hppDout (info, "f_qIn_=" << f_qIn_);
	result.vector () = qOut_ - f_qIn_;
        hppDout (info, "result=" << result);
      }

      void impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const
      {
	jacobian.setZero ();
        size_type iq = 0, iv = 0, nq, nv;
        std::size_t rank = 0;
        impl_compute (result_, arg);
        inputToOutput_->jacobian (Jf_, qIn_);
        hppDout (info, "Jf_=" << std::endl << Jf_);
        // Fill Jacobian by set of lines corresponding to the types of Lie group
        // that compose the outputspace of input to output function.
        segments_t outConfSegments (outputConfIntervals_);
        for (std::vector <LiegroupType>::const_iterator it =
               inputToOutput_->outputSpace ()-> liegroupTypes ().begin ();
             it != inputToOutput_->outputSpace ()-> liegroupTypes ().end ();
             ++it) {
          nq = inputToOutput_->outputSpace ()->nq (rank);
          nv = inputToOutput_->outputSpace ()->nv (rank);
          JacobianVisitor v (qOut_.vector ().segment (iq, nq),
                             f_qIn_.vector ().segment (iq, nq),
                             Jf_.middleRows (iv, nv), outJacobian_ [rank],
                             inJacobian_ [rank], jacobian.middleRows (iv, nv));
          boost::apply_visitor (v, *it);
          iq += nq;
          iv += nv;
          ++rank;
        }
      }

    private:
      void computeJacobianBlocks ()
      {
        segments_t remainingCols (outputDerivIntervals_);
        segments_t cols;
        size_type iv = 0, nv;
        std::size_t rank = 0;
        for (std::vector <LiegroupType>::const_iterator it =
               inputToOutput_->outputSpace ()->liegroupTypes ().begin ();
             it != inputToOutput_->outputSpace ()->liegroupTypes ().end ();
             ++it) {
          nv = inputToOutput_->outputSpace ()->nv (rank);
          cols = BlockIndex::split (remainingCols, nv);
          segments_t rows (1, std::make_pair (iv, nv));
          outJacobian_.push_back (Eigen::MatrixBlocks <false, false>
                                  (rows, cols));
          inJacobian_.push_back (Eigen::MatrixBlocks <false, false>
                                 (rows, inputDerivIntervals_));
          //
          iv += nv;
          ++rank;
        }
      }

      DevicePtr_t robot_;
      DifferentiableFunctionPtr_t inputToOutput_;
      segments_t inputConfIntervals_;
      segments_t inputDerivIntervals_;
      segments_t outputConfIntervals_;
      segments_t outputDerivIntervals_;
      std::vector <Eigen::MatrixBlocks <false, false> > outJacobian_;
      std::vector <Eigen::MatrixBlocks <false, false> > inJacobian_;
      mutable vector_t qIn_;
      mutable LiegroupElement f_qIn_, qOut_;
      mutable LiegroupElement result_;
      // Jacobian of explicit function
      mutable matrix_t Jf_;
    }; // class ImplicitFunction

  } // namespace core
} // namespace hpp

#endif // HPP_CORE_SRC_IMPLICIT_FUNCTION_HH
