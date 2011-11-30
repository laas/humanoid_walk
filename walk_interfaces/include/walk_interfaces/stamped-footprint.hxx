#ifndef WALK_INTERFACE_STAMPED_FOOTPRINT_HXX
# define WALK_INTERFACE_STAMPED_FOOTPRINT_HXX
# include <iostream>

namespace walk
{
  template <typename T>
  std::ostream&
  operator<<(std::ostream& os, const StampedFootprint<T>& sf)
  {
    os
      << "duration:\n"
      << sf.duration
      << "position:\n"
      << sf.position;
    return os;
  }

  template <typename T>
  TimeDuration
  computeFootprintSequenceLength
  (const WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)& sequence)
  {
    typedef WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>) sequence_t;
    typedef typename sequence_t::const_iterator const_iterator_t;

    TimeDuration length;
    const_iterator_t iter = sequence.begin();
    for (; iter != sequence.end(); ++iter)
      length += iter->duration;
    return length;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_STAMPED_FOOTPRINT_HXX
