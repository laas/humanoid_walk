#ifndef WALK_INTERFACE_STAMPED_FOOTPRINT_HH
# define WALK_INTERFACE_STAMPED_FOOTPRINT_HH
# include <Eigen/Core>
# include <boost/date_time.hpp>
# include <walk_interfaces/types.hh>

namespace walk
{
  /// \brief Footprint with its associated timestamp.
  ///
  /// \tparam T Position type.
  template <typename T>
  struct StampedFootprint
  {
    /// \brief Footprint type.
    typedef T footprint_t;

    /// \brief Footprint begin time.
    ///
    /// A footprint begins when the foot reaches the ground.
    Time beginTime;

    /// \brief Footprint duration starting from the begin time.
    ///
    /// A footprint ends when the foot leaves the ground.
    TimeDuration duration;

    /// Footprint position during its existence.
    footprint_t position;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  /// \brief Display a stamped footprint.
  ///
  /// \param stream Output stream.
  /// \param sf Stamped footprint.
  template <typename T>
  std::ostream& operator<<(std::ostream& stream, const StampedFootprint<T>& sf);

  /// \brief 2d stamped footprints.
  typedef StampedFootprint<Footprint2d> StampedFootprint2d;
  /// \brief Vector of 2d stamped footprints.
  typedef WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<Footprint2d>)
  StampedFootprint2dSequence;

  /// \brief Compute a footprint sequence duration.
  ///
  /// A footprint sequence is defined as a sequence of stamped footprints.
  ///
  /// The length (in time) of a footprint sequence is defined by the
  /// duration separating the begin time of the first footprint and the
  /// end time of the last footprint.
  ///
  /// \tparam T Tootprint type.
  template <typename T>
  TimeDuration
  computeFootprintSequenceLength
  (const WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)& sequence);

} // end of namespace walk.

# include <walk_interfaces/stamped-footprint.hxx>
#endif //! WALK_INTERFACE_STAMPED_FOOTPRINT_HH
