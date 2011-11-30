#ifndef WALK_INTERFACE_TYPES_HH
# define WALK_INTERFACE_TYPES_HH
# include <vector>
# include <Eigen/Core>

# include <boost/date_time/posix_time/posix_time.hpp>

# define WALK_INTERFACES_EIGEN_STL_VECTOR(T)				\
  std::vector<T, Eigen::aligned_allocator<std::pair<const int, T> > >

namespace walk
{
  typedef Eigen::Matrix<double, 4, 4> HomogeneousMatrix3d;
  typedef Eigen::Matrix<double, 3, 3> HomogeneousMatrix2d;
  typedef Eigen::Vector2d Vector2d;
  typedef Eigen::Vector3d Vector3d;

  typedef Eigen::Matrix<double, 3, 1> Footprint2d;
  typedef Eigen::VectorXd Posture;

  typedef boost::posix_time::ptime Time;
  typedef boost::posix_time::time_duration TimeDuration;

  typedef WALK_INTERFACES_EIGEN_STL_VECTOR(Footprint2d) Footprint2dSequence;


} // end of namespace walk.

#endif //! WALK_INTERFACE_TYPES_HH
