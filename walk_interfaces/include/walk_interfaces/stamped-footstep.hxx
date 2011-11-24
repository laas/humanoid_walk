#ifndef WALK_INTERFACE_STAMPED_FOOTSTEP_HXX
# define WALK_INTERFACE_STAMPED_FOOTSTEP_HXX
# include <iostream>

namespace walk
{
  template <typename T>
  std::ostream&
  operator<<(std::ostream& os, const StampedFootstep<T>& sf)
  {
    os
      << "duration:\n"
      << sf.duration
      << "footstep:\n"
      << sf.footstep;
    return os;
  }

  template <typename T>
  TimeDuration
  computeFootstepSequenceLength
  (const WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootstep<T>)& sequence)
  {
    typedef WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootstep<T>) sequence_t;
    typedef typename sequence_t::const_iterator const_iterator_t;

    TimeDuration length;
    const_iterator_t iter = sequence.begin();
    for (; iter != sequence.end(); ++iter)
      length += iter->duration;
    return length;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_STAMPED_FOOTSTEP_HXX
