#ifndef WALK_INTERFACE_STAMPED_POSITION_HH
# define WALK_INTERFACE_STAMPED_POSITION_HH
# include <iostream>
# include <boost/date_time.hpp>
# include <boost/date_time/posix_time/posix_time_io.hpp>
# include <Eigen/Core>
# include <boost/date_time.hpp>
# include <walk_interfaces/types.hh>

namespace walk
{
  /// \brief Position with its associated time duration.
  ///
  /// Used to store an element of a discretized trajectory.  The
  /// discretization step is not supposed to be constant and is store
  /// in the duration field of this class.
  ///
  /// \tparam T Position type.
  template <typename T>
  struct StampedPosition
  {
    /// \brief Value duration.
    TimeDuration duration;
    /// \brief Value.
    T position;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  /// \brief 3d position with timestamp.
  typedef StampedPosition<HomogeneousMatrix3d> StampedPosition3d;
  /// \brief 2d position with timestamp.
  typedef StampedPosition<HomogeneousMatrix2d> StampedPosition2d;
  /// \brief 2d vector with timestamp.
  typedef StampedPosition<Eigen::Vector2d> StampedVector2d;
  /// \brief 3d vector with timestamp.
  typedef StampedPosition<Eigen::Vector3d> StampedVector3d;
  /// \brief Nd vector with timestamp.
  typedef StampedPosition<Eigen::VectorXd> StampedVectorNd;

  /// \brief Display a stamped position.
  ///
  /// \param os Output stream.
  /// \param sp Stamped position.
  template <typename T>
  std::ostream&
  operator<<(std::ostream& os, const StampedPosition<T>& sp)
  {
    os << sp.duration << " " << sp.position;
    return os;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_STAMPED_POSITION_HH
