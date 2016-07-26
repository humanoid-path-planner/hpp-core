#include <hpp/pinocchio/hpp-model/conversions.hh>
#include <hpp/pinocchio/hpp-model/model-loader.hh>

template<typename D>
bool isPermutation(const Eigen::MatrixBase<D> & R)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D,3,3);

  for( int row=0;row<3;row++ )
    for( int col=0;col<3;col++ )
      if ( (std::abs(R(row,col))>1e-6) && (std::abs(std::abs(R(row,col))-1)>1e-6) )
        return false;
  return true;
}

/* Frames in hpp::model are "permuted" (i.e. combination of PI/2 Cartesian rotation)
 * so check here that the two placements fits, under this permutation.
 */
bool isApproxPermutation( hpp::model::Transform3f Mm,
                          hpp::pinocchio::Transform3f Mp )
{
  return  Mm.getTranslation().isApprox( Mp.translation() )
    &&  isPermutation(Eigen::Matrix3d(Mp.rotation().transpose()*Mm.getRotation())) ;
}


BOOST_AUTO_TEST_CASE(convert)
{
  Eigen::VectorXd q = Eigen::VectorXd::Random(20);
  BOOST_CHECK( q.isApprox(m2p::q(p2m::q(q))) );

  Eigen::VectorXd v = Eigen::VectorXd::Random(20);
  Eigen::MatrixXd J = Eigen::MatrixXd::Random(3,20);
  Eigen::MatrixXd R(3,3);
  R << 0,0,1 ,1,0,0, 0,1,0;

  BOOST_CHECK( v.isApprox(p2m::Xq(R)*(m2p::Xq(R)*v)) );
  BOOST_CHECK( J.isApprox((J*p2m::Xq(R))*m2p::Xq(R)) );
  BOOST_CHECK( ((J*m2p::Xq(R))*(p2m::Xq(R)*v)).isApprox(J*v) );

  se3::SE3 Mp = se3::SE3::Random();
  BOOST_CHECK( Mp.isApprox( m2p::SE3(p2m::SE3(Mp)) ) );
}
