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
#include <hpp/core/explicit-numerical-constraint.hh>

namespace hpp {
  namespace core {

    typedef Eigen::Quaternion<value_type> Quat_t;
    typedef Eigen::Map<Quat_t > QuatMap_t;
    typedef Eigen::Map<const Quat_t > QuatConstMap_t;
    typedef hpp::pinocchio::liegroup::VectorSpaceOperation <3, false> R3;
    typedef hpp::pinocchio::liegroup::SpecialOrthogonalOperation <3> SO3;
    typedef hpp::pinocchio::liegroup::CartesianProductOperation <R3, SO3>
    R3xSO3;
    typedef se3::SpecialEuclideanOperation <3> SE3;
    typedef hpp::pinocchio::LiegroupType LiegroupType;

    struct ValueVisitor : public boost::static_visitor <>
    {
      ValueVisitor (vectorIn_t qOut, vectorIn_t f_qIn, const size_type nv) :
        qOut_ (qOut), f_qIn_ (f_qIn), result_ (nv)
      {}

      template <typename LgT> void operator () (const LgT&)
      {
        LgT::difference (f_qIn_, qOut_, result_);
      }

      vectorIn_t qOut_, f_qIn_;
      vector_t result_;
    }; // struct JacobianVisitor

    template <> inline void ValueVisitor::operator () <SE3> (const SE3&)
    {
      Transform3f Mout (
          QuatConstMap_t (qOut_.tail <4>().data()).toRotationMatrix (),
          qOut_.head <3> ());
      Transform3f Mf   (
          QuatConstMap_t (f_qIn_.tail <4>().data()).toRotationMatrix (),
          f_qIn_.head <3> ());
      // \f$Mf^{-1} M_{out}\f$
      Transform3f Mf_inverse_Mout (Mf.inverse () * Mout);

      result_.head <3>() = Mf_inverse_Mout.translation();

      value_type theta;
      constraints::logSO3 (Mf_inverse_Mout.rotation(), theta, result_.tail<3>());
    }

    template <bool GisIdentity> struct JacobianVisitor;

    template <bool GisIdentity, typename LgT> struct JacobianVisitorImpl
    {
      static void run (JacobianVisitor<GisIdentity>& v)
      {
        if (GisIdentity)
          v.outJacobian_.lview (v.result_).setIdentity ();
        else
          v.outJacobian_.lview (v.result_) = v.Jg_;
        v.inJacobian_ .lview (v.result_) = - v.Jf_;
      }
    };

    template <bool GisIdentity> struct JacobianVisitorImpl <GisIdentity, R3xSO3>
    {
      static void run (JacobianVisitor<GisIdentity>& v)
      {
        using Eigen::MatrixBlocks;
        using Eigen::BlockIndex;
        hppDout (info, "result_ = " << std::endl << v.result_);

        assert (v.outJacobian_.nbRows () == 6);
        matrix6_t J_qout (matrix6_t::Identity());

        // Fill R^3 part
        // J_qout.topLeftCorner<3,3>().setIdentity();

        // extract 3 top rows of inJacobian_
        v.inJacobian_.middleRows(0,3).lview (v.result_) = - v.Jf_.template topRows <3> ();
        hppDout (info, "result_ = " << std::endl << v.result_);

        // Fill SO(3) part
        assert (v.qOut_.size () == 7);
        assert (v.f_qIn_.size () == 7);
        matrix3_t R_out (QuatConstMap_t (v.qOut_.template tail <4> ().data())
            .toRotationMatrix ());
        matrix3_t R_f (QuatConstMap_t (v.f_qIn_.template tail <4> ().data())
            .toRotationMatrix ());
        // \f$R_f^T R_{out}\f$
        matrix3_t R_f_T_R_out (R_f.transpose () * R_out);
        matrix3_t Jlog_R_f_T_R_out;
        vector3_t r;
        value_type theta;
        constraints::logSO3 (R_f_T_R_out, theta, r);
        constraints::JlogSO3 (theta, r, Jlog_R_f_T_R_out);

        J_qout.bottomRightCorner<3,3>() = Jlog_R_f_T_R_out;
        // extract 3 bottom rows of inJacobian_
        v.inJacobian_.middleRows(3,3).lview (v.result_) =
          ( - Jlog_R_f_T_R_out * R_out.transpose () * R_f ) 
          * v.Jf_.template bottomRows <3> ();
        hppDout (info, "result_ = " << std::endl << v.result_);

        if (GisIdentity)
          v.outJacobian_.lview (v.result_) = J_qout;
        else
          v.outJacobian_.lview (v.result_) = J_qout * v.Jg_;
        hppDout (info, "result_ = " << std::endl << v.result_);
      }
    };

    template <bool GisIdentity> struct JacobianVisitorImpl <GisIdentity, SE3>
    {
      static void run (JacobianVisitor<GisIdentity>& v)
      {
        assert (v.outJacobian_.nbRows () == 6);
        // extract 3 top rows of inJacobian_
        assert (v.qOut_.size () == 7);
        assert (v.f_qIn_.size () == 7);

        Transform3f Mout (
            QuatConstMap_t (v.qOut_.template tail <4>().data()).toRotationMatrix (),
            v.qOut_.template head <3> ());
        Transform3f Mf (
            QuatConstMap_t (v.f_qIn_.template tail <4> ().data()).toRotationMatrix (),
            v.f_qIn_.template head <3> ());
        // \f$Mf^{-1} M_{out}\f$
        Transform3f Mf_inverse_Mout (Mf.inverse () * Mout);
        matrix6_t Jout (matrix6_t::Zero());
        matrix3_t Jlog;
        vector3_t r;
        value_type theta;
        constraints::logSO3 (Mf_inverse_Mout.rotation(), theta, r);
        constraints::JlogSO3 (theta, r, Jlog);
        Jout.    topLeftCorner<3,3>() = Mf_inverse_Mout.rotation();
        Jout.bottomRightCorner<3,3>() = Jlog;

        if (GisIdentity)
          v.outJacobian_.lview (v.result_) = Jout;
        else
          v.outJacobian_.lview (v.result_) = Jout * v.Jg_;

        Jout.    topLeftCorner<3,3>().setIdentity();
        JointJacobian_t inJ (6, v.Jf_.cols ());
        inJ.topRows <3> ().noalias() =
          se3::skew (Mf_inverse_Mout.translation()) * v.Jf_.template bottomRows <3> ();
        inJ.topRows <3> ().noalias() -= v.Jf_.template topRows <3> ();
        inJ.bottomRows <3> ().noalias() = - Mf_inverse_Mout.rotation().transpose() * v.Jf_.template bottomRows <3> ();

        v.inJacobian_.lview (v.result_) = Jout * inJ;
      }
    };

    template <bool GisIdentity>
    struct JacobianVisitor : public boost::static_visitor <>
    {
      JacobianVisitor (vectorIn_t qOut, vectorIn_t f_qIn,
                       matrixIn_t Jg, matrixIn_t Jf,
                       const Eigen::MatrixBlocks <false, false>&
                       outJacobian, const Eigen::MatrixBlocks <false, false>&
                       inJacobian, matrixOut_t result) :
        qOut_ (qOut), f_qIn_ (f_qIn), Jg_ (Jg), Jf_ (Jf),
        outJacobian_ (outJacobian), inJacobian_ (inJacobian), result_ (result)
      {}

      template <typename LgT> inline void operator () (const LgT&)
      {
        JacobianVisitorImpl<GisIdentity, LgT>::run (*this);
      }

      vectorIn_t qOut_, f_qIn_;
      matrixIn_t Jg_, Jf_;
      const Eigen::MatrixBlocks <false, false>& outJacobian_;
      const Eigen::MatrixBlocks <false, false>& inJacobian_;
      matrixOut_t result_;
    }; // struct JacobianVisitor

    template <bool GisIdentity> class ImplicitFunction;

    /// Function of the form f (q) = q2 - g (q1)
    ///
    /// where
    ///  \li q2 is a vector composed of a subset of configuration variables of
    ///      q,
    ///  \li q1 is the vector composed of the other configuration variables of
    ///      q,
    ///  g is a differentiable function with values in  a Lie group.
    template <bool GisIdentity>
    class ImplicitFunction : public DifferentiableFunction
    {
    public:
      typedef boost::shared_ptr <ImplicitFunction> Ptr_t;

      static Ptr_t create
      (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
       const segments_t& inputConf, const segments_t& inputVelocity,
       const segments_t& outputConf, const segments_t& outputVelocity)
      {
        assert (GisIdentity);
	ImplicitFunction* ptr = new ImplicitFunction
	  (robot, function, DifferentiableFunctionPtr_t(), inputConf, inputVelocity, outputConf,
           outputVelocity);
	return Ptr_t (ptr);
      }

      static Ptr_t create
      (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
       const DifferentiableFunctionPtr_t& g,
       const segments_t& inputConf, const segments_t& inputVelocity,
       const segments_t& outputConf, const segments_t& outputVelocity)
      {
        assert (!GisIdentity);
	ImplicitFunction* ptr = new ImplicitFunction
	  (robot, function, g, inputConf, inputVelocity, outputConf,
           outputVelocity);
	return Ptr_t (ptr);
      }

      /// Get function that maps input variables to output variables
      const DifferentiableFunctionPtr_t& inputToOutput () const
      {
        return inputToOutput_;
      }

    protected:
      ImplicitFunction (const DevicePtr_t& robot,
			const DifferentiableFunctionPtr_t& function,
                        const DifferentiableFunctionPtr_t& g,
			const segments_t& inputConf,
			const segments_t& inputVelocity,
                        const segments_t& outputConf,
			const segments_t& outputVelocity)
	: DifferentiableFunction (robot->configSize (), robot->numberDof (),
				  LiegroupSpace::Rn
                                  (function->outputSpace ()->nv ()),
                                  "implicit " + function->name()),
	  robot_ (robot), inputToOutput_ (function),
          inputConfIntervals_ (inputConf),
	  inputDerivIntervals_ (inputVelocity),
          outputConfIntervals_ (outputConf),
	  outputDerivIntervals_ (outputVelocity), outJacobian_ (),
          inJacobian_ (), gData_ (g), f_qIn_ (function->outputSpace ()),
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

        inputConfIntervals_ .lview (activeParameters_.matrix()).setConstant(true);
        inputDerivIntervals_.lview (activeDerivativeParameters_.matrix()).setConstant(true);
        outputConfIntervals_ .lview (activeParameters_.matrix()).setConstant(true);
        outputDerivIntervals_.lview (activeDerivativeParameters_.matrix()).setConstant(true);
      }

      /// Compute q_{output} - f (q_{input})
      void impl_compute (LiegroupElement& result, vectorIn_t argument) const
      {
        using Eigen::MatrixBlocks;
        hppDout (info, "argument=" << argument.transpose ());
        // Store q_{output} in result
        qOut_.vector () = outputConfIntervals_.rview (argument);
        hppDout (info, "qOut_=" << qOut_);
        gData_.computeValue(qOut_);
        const LiegroupElement& g_qOut (gData_.value(qOut_));
        hppDout (info, "g_qOut_=" << g_qOut);
        // fill in q_{input}
        qIn_ = inputConfIntervals_.rview (argument);
        hppDout (info, "qIn_=" << qIn_);
        // compute  f (q_{input}) -> output_
	inputToOutput_->value (f_qIn_, qIn_);
        hppDout (info, "f_qIn_=" << f_qIn_);
        // Fill result
        size_type iq = 0, iv = 0, nq, nv;
        std::size_t rank = 0;
        for (std::vector <LiegroupType>::const_iterator it =
               f_qIn_.space()-> liegroupTypes ().begin ();
             it != f_qIn_.space()-> liegroupTypes ().end ();
             ++it) {
          nq = f_qIn_.space()->nq (rank);
          nv = f_qIn_.space()->nv (rank);
          ValueVisitor v (g_qOut.vector ().segment (iq, nq),
                          f_qIn_.vector ().segment (iq, nq),
                          nv);
          boost::apply_visitor (v, *it);
          result.vector ().segment (iv, nv) = v.result_;
          iq += nq;
          iv += nv;
          ++rank;
        }
        hppDout (info, "result=" << result);
      }

      void impl_jacobian (matrixOut_t jacobian, vectorIn_t arg) const
      {
        typedef JacobianVisitor<GisIdentity> JV_t;

	jacobian.setZero ();
        size_type iq = 0, iv = 0, nq, nv;
        std::size_t rank = 0;
        impl_compute (result_, arg);
        inputToOutput_->jacobian (Jf_, qIn_);
        hppDout (info, "Jf_=" << std::endl << Jf_);
        const matrix_t& Jg (gData_.jacobian (qOut_));
        const LiegroupElement& g_qOut (gData_.value(qOut_));
        // Fill Jacobian by set of lines corresponding to the types of Lie group
        // that compose the outputspace of input to output function.
        for (std::vector <LiegroupType>::const_iterator it =
               inputToOutput_->outputSpace ()-> liegroupTypes ().begin ();
             it != inputToOutput_->outputSpace ()-> liegroupTypes ().end ();
             ++it) {
          nq = inputToOutput_->outputSpace ()->nq (rank);
          nv = inputToOutput_->outputSpace ()->nv (rank);
          JV_t v (g_qOut.vector ().segment (iq, nq),
                  f_qIn_.vector ().segment (iq, nq),
                  Jg, Jf_.middleRows (iv, nv), outJacobian_ [rank],
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
        segments_t remainingCols (outputDerivIntervals_.indices());
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
                                 (rows, inputDerivIntervals_.indices()));
          //
          iv += nv;
          ++rank;
        }
      }

      struct GenericGData {
        DifferentiableFunctionPtr_t g_;
        mutable LiegroupElement g_qOut_;
        mutable matrix_t Jg_;

        GenericGData (const DifferentiableFunctionPtr_t& g)
          : g_ (g), g_qOut_ (g->outputSpace()),
          Jg_ (g->outputSpace()->nv(), g->inputDerivativeSize())
        {}
        void computeValue (const LiegroupElement& qOut) const
        {
          g_->value (g_qOut_, qOut.vector());
        }
        const LiegroupElement& value (const LiegroupElement&) const
        {
          return g_qOut_;
        }
        matrix_t& jacobian (const LiegroupElement& qOut) const
        {
          g_->jacobian (Jg_, qOut.vector());
          return Jg_;
        }
      };

      struct IdentityData {
        mutable matrix_t Jg_;
        IdentityData (const DifferentiableFunctionPtr_t&) {}
        void computeValue (const LiegroupElement&) const {}
        const LiegroupElement& value (const LiegroupElement& qOut) const { return qOut; }
        const matrix_t& jacobian (const LiegroupElement&) const { return Jg_; }
      };

      typedef typename boost::conditional<GisIdentity, IdentityData, GenericGData>::type GData;

      DevicePtr_t robot_;
      DifferentiableFunctionPtr_t inputToOutput_;
      Eigen::RowBlockIndices inputConfIntervals_;
      Eigen::RowBlockIndices inputDerivIntervals_;
      Eigen::RowBlockIndices outputConfIntervals_;
      Eigen::RowBlockIndices outputDerivIntervals_;
      std::vector <Eigen::MatrixBlocks <false, false> > outJacobian_;
      std::vector <Eigen::MatrixBlocks <false, false> > inJacobian_;
      GData gData_;
      mutable vector_t qIn_;
      mutable LiegroupElement f_qIn_, qOut_;
      mutable LiegroupElement result_;
      // Jacobian of explicit function
      mutable matrix_t Jf_;
    }; // class ImplicitFunction

    typedef ImplicitFunction<true > BasicImplicitFunction;
    typedef ImplicitFunction<false> GenericImplicitFunction;

  } // namespace core
} // namespace hpp

#endif // HPP_CORE_SRC_IMPLICIT_FUNCTION_HH
