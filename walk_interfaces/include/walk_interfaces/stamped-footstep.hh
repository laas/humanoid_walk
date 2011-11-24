#ifndef WALK_INTERFACE_STAMPED_FOOTSTEP_HH
# define WALK_INTERFACE_STAMPED_FOOTSTEP_HH
# include <Eigen/Core>
# include <boost/date_time.hpp>
# include <walk_interfaces/types.hh>

namespace walk
{
  template <typename T>
  struct StampedFootstep
  {
    typedef T footstep_t;

    TimeDuration duration;
    footstep_t position;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  template <typename T>
  std::ostream& operator<<(std::ostream&, const StampedFootstep<T>&);

  typedef StampedFootstep<Footstep2d> StampedFootstep2d;
  typedef WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootstep<Footstep2d>)
  StampedFootstep2dSequence;

  template <typename T>
  TimeDuration
  computeFootstepSequenceLength
  (const WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootstep<T>)& sequence);

} // end of namespace walk.

# include <walk_interfaces/stamped-footstep.hxx>
#endif //! WALK_INTERFACE_STAMPED_FOOTSTEP_HH
