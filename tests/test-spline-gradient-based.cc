#define BOOST_TEST_MODULE spline_gradient_based
#include <boost/mpl/list.hpp>
#include <boost/test/included/unit_test.hpp>
#include <hpp/core/path-optimization/spline-gradient-based.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/util/debug.hh>

using namespace hpp::pinocchio;
using namespace hpp::core;

DevicePtr_t createNDoFRobot(int ndof) {
  std::ostringstream oss;
  oss << "<robot name='test'>"
      << "<link name='link0'/>";
  for (int i = 0; i < ndof; ++i) {
    oss << "<joint name='joint" << i << "' type='prismatic'>"
        << "<parent link='link" << i << "'/>"
        << "<child  link='link" << i + 1 << "'/>"
        << "<limit effort='30' velocity='1.0' lower='-4' upper='4'/>"
        << "</joint>"
        << "<link name='link" << i + 1 << "'/>";
  }
  oss << "</robot>";
  std::string urdf(oss.str());

  DevicePtr_t robot = Device::create("test");
  urdf::loadModelFromString(robot, 0, "", "anchor", urdf, "");
  return robot;
}

typedef pathOptimization::SplineGradientBased<path::BernsteinBasis, 3>
    SplineGradientBased_bezier3;
typedef pathOptimization::SplineGradientBased<path::BernsteinBasis, 5>
    SplineGradientBased_bezier5;
typedef pathOptimization::SplineGradientBased<path::BernsteinBasis, 7>
    SplineGradientBased_bezier7;

template <typename OptimizerType>
void test_spline_gradient_based(size_type costOrder) {
  DevicePtr_t dev = createNDoFRobot(1);
  ProblemPtr_t problem = Problem::create(dev);
  problem->setParameter("SplineGradientBased/returnOptimum", Parameter(true));
  problem->setParameter("SplineGradientBased/costOrder", Parameter(costOrder));

  typename OptimizerType::Ptr_t opt = OptimizerType::create(problem);

  PathVectorPtr_t pv = PathVector::create(dev->configSize(), dev->numberDof());

  Configuration_t q0(dev->configSize()), q1(dev->configSize());
  q0 << 0.0;
  vector_t subpathTimes(vector_t::LinSpaced(4, 0.0, 3.0));
  for (int i = 1; i < subpathTimes.size(); ++i) {
    q1 << subpathTimes[i];
    pv->appendPath(StraightPath::create(dev, q0, q1, (q1 - q0)[0]));
    q0 = q1;
  }

  PathVectorPtr_t optPath = opt->optimize(pv);

  vector_t ts(vector_t::LinSpaced(subpathTimes.size() * 10, 0.0, 3.0));
  Configuration_t qi(dev->configSize());
  vector_t vi(dev->numberDof()), ai(dev->numberDof());
  std::cout << "d_order" << OptimizerType::SplineOrder << "_cost" << costOrder
            << "=[";
  for (int i = 0; i < ts.size(); ++i) {
    optPath->eval(qi, ts[i]);
    optPath->derivative(vi, ts[i], 1);
    optPath->derivative(ai, ts[i], 2);
    std::cout << '[' << ts[i] << ',' << qi << ',' << vi << ',' << ai << "],";
  }
  std::cout << "]\n";
}

typedef boost::mpl::list<SplineGradientBased_bezier3,
                         SplineGradientBased_bezier5,
                         SplineGradientBased_bezier7>
    SplineGradientBaseds_t;

BOOST_AUTO_TEST_CASE_TEMPLATE(spline_gradient_based, SplineGradientBasedType,
                              SplineGradientBaseds_t) {
  hpp::debug::setVerbosityLevel(50);
  test_spline_gradient_based<SplineGradientBasedType>(1);
  test_spline_gradient_based<SplineGradientBasedType>(2);
  test_spline_gradient_based<SplineGradientBasedType>(3);
}

BOOST_AUTO_TEST_CASE(fake_spline_gradient_based_get_python_plot_function) {
  std::cout << R"(
import numpy as np
import matplotlib.pyplot as plt

def plot_q(dd):
  d = np.array(dd)
  plt.title("position")
  plt.plot(d[:,0], d[:,1])
  plt.plot([d[0,0], d[-1,0]], [d[0,1], d[-1,1]])

  plt.show()

def plot_qva(dd):
  d = np.array(dd)
  plt.subplot(3,1,1)
  plt.title("position")
  plt.plot(d[:,0], d[:,1])

  plt.subplot(3,1,2)
  plt.title("velocity")
  plt.plot(d[:,0], d[:,2])

  plt.subplot(3,1,3)
  plt.title("acceleration")
  plt.plot(d[:,0], d[:,3])

  plt.show()
)";
}
