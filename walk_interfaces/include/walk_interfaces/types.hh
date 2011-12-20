#ifndef WALK_INTERFACE_TYPES_HH
# define WALK_INTERFACE_TYPES_HH
# include <vector>
# include <Eigen/Core>

# include <boost/date_time/posix_time/posix_time.hpp>

/// \brief Define a vector of elements containing Eigen objects.
///
/// The STL containers containing Eigen matrices must use a specific
/// allocator. This macro wraps the type definition.
///
/// It is not possible to use a typedef in this case as templated
/// typedef are not supported in C++03 (unlike C++11).
# define WALK_INTERFACES_EIGEN_STL_VECTOR(T)				\
  std::vector<T, Eigen::aligned_allocator<std::pair<const int, T> > >

namespace walk
{
  /// \brief 3D homogeneous matrix.
  ///
  /// 4x4 matrix representing an element of \f$SO(3)\f$.
  typedef Eigen::Matrix<double, 4, 4> HomogeneousMatrix3d;

  /// \brief 2D homogeneous matrix.
  ///
  /// 3x3 matrix representing an element of \f$SO(2)\f$.
  typedef Eigen::Matrix<double, 3, 3> HomogeneousMatrix2d;

  /// \brief 2D vector.
  typedef Eigen::Vector2d Vector2d;
  /// \brief 3D vector.
  typedef Eigen::Vector3d Vector3d;

  /// \brief 2D footprint.
  typedef Eigen::Matrix<double, 3, 1> Footprint2d;

  /// \brief Posture (Eigen vector).
  typedef Eigen::VectorXd Posture;

  /// \brief Time.
  typedef boost::posix_time::ptime Time;
  /// \brief Duration.
  typedef boost::posix_time::time_duration TimeDuration;

  /// \brief Vector of 2d footprints.
  typedef WALK_INTERFACES_EIGEN_STL_VECTOR(Footprint2d) Footprint2dSequence;

} // end of namespace walk.

#endif //! WALK_INTERFACE_TYPES_HH
