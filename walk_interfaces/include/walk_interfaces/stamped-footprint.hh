#ifndef WALK_INTERFACE_STAMPED_FOOTPRINT_HH
# define WALK_INTERFACE_STAMPED_FOOTPRINT_HH
# include <Eigen/Core>
# include <boost/date_time.hpp>
# include <walk_interfaces/types.hh>

namespace walk
{
  template <typename T>
  struct StampedFootprint
  {
    typedef T footprint_t;

    Time beginTime;
    TimeDuration duration;
    footprint_t position;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  template <typename T>
  std::ostream& operator<<(std::ostream&, const StampedFootprint<T>&);

  typedef StampedFootprint<Footprint2d> StampedFootprint2d;
  typedef WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<Footprint2d>)
  StampedFootprint2dSequence;

  template <typename T>
  TimeDuration
  computeFootprintSequenceLength
  (const WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)& sequence);

} // end of namespace walk.

# include <walk_interfaces/stamped-footprint.hxx>
#endif //! WALK_INTERFACE_STAMPED_FOOTPRINT_HH
