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
  template <typename T>
  struct StampedPosition
  {
    TimeDuration duration;
    T position;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  typedef StampedPosition<HomogeneousMatrix3d> StampedPosition3d;
  typedef StampedPosition<HomogeneousMatrix2d> StampedPosition2d;
  typedef StampedPosition<Eigen::Vector2d> StampedVector2d;
  typedef StampedPosition<Eigen::Vector3d> StampedVector3d;
  typedef StampedPosition<Eigen::VectorXd> StampedVectorNd;

  template <typename T>
  std::ostream&
  operator<<(std::ostream& os, const StampedPosition<T>& sp)
  {
    os << sp.duration << " " << sp.position;
    return os;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_STAMPED_POSITION_HH
