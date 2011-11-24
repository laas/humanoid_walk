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
  struct StampedPosition
  {
    TimeDuration duration;
    HomogeneousMatrix position;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  std::ostream&
  operator<<(std::ostream& os, const StampedPosition& sp)
  {
    os << sp.duration << " " << sp.position;
    return os;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_STAMPED_POSITION_HH
