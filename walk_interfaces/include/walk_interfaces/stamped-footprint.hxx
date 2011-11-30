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
      << "period:\n"
      << sf.period
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

    // If sequence is empty return zero duration.
    if (sequence.size () != 0)
      {
	const_iterator_t beginIter = sequence.begin();
	const_iterator_t endIter = sequence.end(); --endIter;
	length = (beginIter->period.span (endIter->period)).length ();
      }

    return length;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_STAMPED_FOOTPRINT_HXX
