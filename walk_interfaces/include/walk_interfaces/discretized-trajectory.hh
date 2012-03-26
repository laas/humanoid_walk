#ifndef WALK_INTERFACE_DISCRETIZED_TRAJECTORY_HH
# define WALK_INTERFACE_DISCRETIZED_TRAJECTORY_HH
# include <iosfwd>
# include <vector>

# include <walk_interfaces/types.hh>
# include <walk_interfaces/stamped-position.hh>

namespace walk
{
  /// \brief A trajectory is a function which takes a time as its
  /// input and return a value.
  ///
  /// This class is used to store reference trajectories. It is
  /// templated by the trajectory output type. E.g. \f$SO(3)\f$,
  /// \f$SO(2)\f$, \f$R^n\f$...
  ///
  /// It relies on discretized data to store the values at different
  /// point of time. No data interpolation is realized in this class.
  ///
  /// \tparam T Trajectory output type.
  template <typename T>
  class DiscretizedTrajectory
  {
  public:
    /// \brief Trajectory element.
    typedef T element_t;
    /// \brief Vector of trajectory elements.
    typedef WALK_INTERFACES_EIGEN_STL_VECTOR(T) data_t;

    /// \name Constructors and destructor.
    /// \{

    /// \brief Default constructor.
    explicit DiscretizedTrajectory();
    /// \brief Copy constructor.
    explicit DiscretizedTrajectory(const DiscretizedTrajectory<T>&);
    /// \brief Destructor.
    ~DiscretizedTrajectory();

    /// \}

    /// \brief Assignment operator.
    DiscretizedTrajectory<T>& operator=(const DiscretizedTrajectory<T>&);

    /// \brief Trajectory data getter.
    data_t& data();

    /// \brief Trajectory data getter (const).
    const data_t& data() const;

    /// \brief Compute the trajectory length.
    ///
    /// Algorithm complexity: linear in the number of trajectory
    /// elements.
    TimeDuration computeLength() const;

  private:
    /// \brief Internal data.
    data_t data_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  /// \brief DiscretizedTrajectory in \f$SO(3)\f$.
  typedef DiscretizedTrajectory<StampedPosition3d> DiscretizedTrajectory3d;
  /// \brief DiscretizedTrajectory in \f$SO(2)\f$.
  typedef DiscretizedTrajectory<StampedPosition2d> DiscretizedTrajectory2d;
  /// \brief DiscretizedTrajectory in \f$R^2\f$.
  typedef DiscretizedTrajectory<StampedVector2d> DiscretizedTrajectoryV2d;
  /// \brief DiscretizedTrajectory in \f$R^3\f$.
  typedef DiscretizedTrajectory<StampedVector3d> DiscretizedTrajectoryV3d;
  /// \brief DiscretizedTrajectory in \f$R^n\f$.
  typedef DiscretizedTrajectory<StampedVectorNd> DiscretizedTrajectoryNd;

  /// \brief Display a trajectory.
  ///
  /// \param stream Output stream
  /// \param trajectory Displayed trajectory
  template <typename T>
  std::ostream& operator<<(std::ostream& stream,
			   const DiscretizedTrajectory<T>& trajectory);

} // end of namespace walk.

# include <walk_interfaces/discretized-trajectory.hxx>
#endif //! WALK_INTERFACE_DISCRETIZED_TRAJECTORY_HH
