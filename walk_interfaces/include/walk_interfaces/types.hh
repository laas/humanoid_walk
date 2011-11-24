#ifndef WALK_INTERFACE_TYPES_HH
# define WALK_INTERFACE_TYPES_HH
# include <vector>
# include <Eigen/Core>

# include <boost/date_time/posix_time/posix_time_duration.hpp>

# define WALK_INTERFACES_EIGEN_STL_VECTOR(T)				\
  std::vector<T, Eigen::aligned_allocator<std::pair<const int, T> > >

namespace walk
{
  typedef Eigen::Matrix<double, 4, 4> HomogeneousMatrix;
  typedef Eigen::Matrix<double, 3, 1> Footstep2d;
  typedef Eigen::VectorXd Posture;

  typedef boost::posix_time::time_duration TimeDuration;

  typedef WALK_INTERFACES_EIGEN_STL_VECTOR(Footstep2d) Footstep2dSequence;


} // end of namespace walk.

#endif //! WALK_INTERFACE_TYPES_HH
