#ifndef WALK_INTERFACE_DISCRETIZED_TRAJECTORY_HXX
# define WALK_INTERFACE_DISCRETIZED_TRAJECTORY_HXX
# include <iostream>

namespace walk
{
  template <typename T>
  DiscretizedTrajectory<T>::DiscretizedTrajectory()
    : data_()
  {}

  template <typename T>
  DiscretizedTrajectory<T>::DiscretizedTrajectory
  (const DiscretizedTrajectory<T>& gamma)
    : data_(gamma.data_)
  {}

  template <typename T>
  DiscretizedTrajectory<T>::~DiscretizedTrajectory()
  {}

  template <typename T>
  DiscretizedTrajectory<T>&
  DiscretizedTrajectory<T>::operator=(const DiscretizedTrajectory<T>& gamma)
  {
    if (&gamma == this)
      return *this;
    this->data_ = gamma.data_;
    return *this;
  }

  template <typename T>
  typename DiscretizedTrajectory<T>::data_t&
  DiscretizedTrajectory<T>::data()
  {
    return data_;
  }

  template <typename T>
  const typename DiscretizedTrajectory<T>::data_t&
  DiscretizedTrajectory<T>::data() const
  {
    return data_;
  }

  template <typename T>
  TimeDuration
  DiscretizedTrajectory<T>::computeLength() const
  {
    TimeDuration length;
    typename data_t::const_iterator iter = data_.begin ();
    for (; iter != data_.end(); ++iter)
      length += iter->duration;
    return length;
  }

  template <typename T>
  std::ostream&
  operator<<(std::ostream& os, const DiscretizedTrajectory<T>& gamma)
  {
    typename DiscretizedTrajectory<T>::data_t::const_iterator iter =
      gamma.data().begin ();
    for (; iter != gamma.data().end(); ++iter)
      os << (*iter) << std::endl;
    return os;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_DISCRETIZED_TRAJECTORY_HXX
