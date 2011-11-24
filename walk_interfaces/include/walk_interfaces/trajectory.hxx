#ifndef WALK_INTERFACE_STAMPED_TRAJECTORY_HXX
# define WALK_INTERFACE_STAMPED_TRAJECTORY_HXX
# include <iostream>

namespace walk
{
  template <typename T>
  Trajectory<T>::Trajectory()
    : data_()
  {}

  template <typename T>
  Trajectory<T>::Trajectory(const Trajectory<T>& gamma)
    : data_(gamma.data_)
  {}

  template <typename T>
  Trajectory<T>::~Trajectory()
  {}

  template <typename T>
  Trajectory<T>&
  Trajectory<T>::operator=(const Trajectory<T>& gamma)
  {
    if (&gamma == this)
      return *this;
    this->data_ = gamma.data_;
    return *this;
  }

  template <typename T>
  typename Trajectory<T>::data_t&
  Trajectory<T>::data()
  {
    return data_;
  }

  template <typename T>
  const typename Trajectory<T>::data_t&
  Trajectory<T>::data() const
  {
    return data_;
  }

  template <typename T>
  TimeDuration
  Trajectory<T>::computeLength() const
  {
    TimeDuration length;
    typename data_t::const_iterator iter = data_.begin ();
    for (; iter != data_.end(); ++iter)
      length += iter->duration;
    return length;
  }

  template <typename T>
  std::ostream&
  operator<<(std::ostream& os, const Trajectory<T>& gamma)
  {
    typename Trajectory<T>::data_t::const_iterator iter =
      gamma.data().begin ();
    for (; iter != gamma.data().end(); ++iter)
      os << (*iter) << std::endl;
    return os;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_STAMPED_TRAJECTORY_HXX
