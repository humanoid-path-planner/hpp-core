//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_BOUNDING_BOX_HH
# define HPP_CORE_BOUNDING_BOX_HH

# include <pinocchio/multibody/geometry.hpp>

# include <hpp/pinocchio/fwd.hh>

namespace hpp {
  namespace core {
    // using pinocchio::GeomModel;
    using pinocchio::GeomData;

    namespace {
      fcl::AABB transform (const fcl::Transform3f& t, const fcl::AABB& in)
      {
        fcl::AABB out;
        fcl::Vec3f corner;
        for (std::size_t cornerIdx = 0; cornerIdx < 7; ++cornerIdx) {
          for (std::size_t i = 0; i < 3; ++i)
            corner[i] = (cornerIdx&(1 << i) ? in.min_[i] : in.max_[i]);
          out += t.transform(corner);
        }
        return out;
      }
    }

    fcl::AABB computeBoundingBox (GeomData& data)
    {
      if (data.collisionObjects.empty()) return fcl::AABB (fcl::Vec3f(0,0,0));
      // Use AABB as a bounding sphere...
      // data.collisionObjects[i].computeAABB();
      // aabb += data.collisionObjects[i].getAABB();
      fcl::AABB aabb = transform(
            data.collisionObjects[0].getTransform(),
            data.collisionObjects[0].collisionGeometry()->aabb_local);
      for (std::size_t i = 1; i < data.collisionObjects.size(); ++i)
      {
        aabb += transform (
            data.collisionObjects[i].getTransform(),
            data.collisionObjects[i].collisionGeometry()->aabb_local);
      }
      return aabb;
    }


    struct SetJointBoundFromAABB : public se3::fusion::JointModelVisitor<SetJointBoundFromAABB>
    {
      typedef boost::fusion::vector<const fcl::AABB &,
              se3::Model&> ArgsType;

      JOINT_MODEL_VISITOR_INIT(SetJointBoundFromAABB);

      template<typename JointModel>
        static fcl::AABB common(const se3::JointModelBase<JointModel> & jmodel,
            const fcl::AABB & aabb,
            const se3::Model& model)
        {
          if (model.parents[jmodel.id()] != 0)
            throw std::logic_error ("The parent of the joint must be universe");
          fcl::Transform3f t = se3::toFclTransform3f (model.jointPlacements[jmodel.id()].inverse());
          return fcl::translate(aabb, t.getTranslation());
        }

      template<typename JointModel>
        static void algo(const se3::JointModelBase<JointModel> &, const fcl::AABB & , se3::Model& )
        {
          throw std::invalid_argument("Set bounds from bounding box is no supported for this type of joints.");
        }
    };

    template<>
      void SetJointBoundFromAABB::algo(const se3::JointModelBase<se3::JointModelFreeFlyer> & jmodel,
          const fcl::AABB & aabb,
          se3::Model& model)
      {
        fcl::AABB box = common(jmodel, aabb, model);

        const se3::Index& iq = jmodel.idx_q();
        model.upperPositionLimit[iq+0] = box.max_[0];
        model.upperPositionLimit[iq+1] = box.max_[1];
        model.upperPositionLimit[iq+2] = box.max_[2];
        model.lowerPositionLimit[iq+0] = box.min_[0];
        model.lowerPositionLimit[iq+1] = box.min_[1];
        model.lowerPositionLimit[iq+2] = box.min_[2];
      }

    template<>
      void SetJointBoundFromAABB::algo(const se3::JointModelBase<se3::JointModelPlanar> & jmodel,
          const fcl::AABB & aabb,
          se3::Model& model)
      {
        fcl::AABB box = common(jmodel, aabb, model);

        const se3::Index& iq = jmodel.idx_q();
        model.upperPositionLimit[iq+0] = box.max_[0];
        model.upperPositionLimit[iq+1] = box.max_[1];
        model.lowerPositionLimit[iq+0] = box.min_[0];
        model.lowerPositionLimit[iq+1] = box.min_[1];
      }
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_BOUNDING_BOX_HH
