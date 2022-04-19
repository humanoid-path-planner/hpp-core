#include "basic.hh"
//#include "k-d-tree.hh"

#include <hpp/core/distance.hh>
#include <hpp/util/serialization.hh>

BOOST_CLASS_EXPORT(hpp::core::nearestNeighbor::Basic)
// BOOST_CLASS_EXPORT(hpp::core::nearestNeighbor::KDTree)

namespace hpp {
namespace core {

template <typename Archive>
inline void NearestNeighbor::serialize(Archive& ar,
                                       const unsigned int version) {
  (void)version;
  (void)ar;
}

HPP_SERIALIZATION_IMPLEMENT(NearestNeighbor);

namespace nearestNeighbor {

template <typename Archive>
inline void Basic::serialize(Archive& ar, const unsigned int version) {
  (void)version;
  ar& boost::serialization::make_nvp(
      "base", boost::serialization::base_object<NearestNeighbor>(*this));
  ar& boost::serialization::make_nvp("distance_",
                                     const_cast<DistancePtr_t&>(distance_));
}

HPP_SERIALIZATION_IMPLEMENT(Basic);

/*
template <typename Archive>
inline void KDTree::serialize(Archive& ar, const unsigned int version)
{
  throw std::logic_error("nearestNeighbor::KDTree not serializable.");
  (void) version;
  ar & boost::serialization::make_nvp("base",
boost::serialization::base_object<NearestNeighbor>(*this)); ar &
BOOST_SERIALIZATION_NVP(distance_);
}

HPP_SERIALIZATION_IMPLEMENT(KDTree);
*/

}  // namespace nearestNeighbor
}  // namespace core
}  // namespace hpp
