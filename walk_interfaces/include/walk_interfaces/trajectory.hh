#ifndef WALK_INTERFACE_STAMPED_TRAJECTORY_HH
# define WALK_INTERFACE_STAMPED_TRAJECTORY_HH
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
  class Trajectory
  {
  public:
    /// \brief Trajectory element.
    typedef T element_t;
    /// \brief Vector of trajectory elements.
    typedef WALK_INTERFACES_EIGEN_STL_VECTOR(T) data_t;

    /// \name Constructors and destructor.
    /// \{

    /// \brief Default constructor.
    explicit Trajectory();
    /// \brief Copy constructor.
    explicit Trajectory(const Trajectory<T>&);
    /// \brief Destructor.
    ~Trajectory();

    /// \}

    /// \brief Assignment operator.
    Trajectory<T>& operator=(const Trajectory<T>&);

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

  /// \brief Trajectory in \f$SO(3)\f$.
  typedef Trajectory<StampedPosition3d> Trajectory3d;
  /// \brief Trajectory in \f$SO(2)\f$.
  typedef Trajectory<StampedPosition2d> Trajectory2d;
  /// \brief Trajectory in \f$R^2\f$.
  typedef Trajectory<StampedVector2d> TrajectoryV2d;
  /// \brief Trajectory in \f$R^3\f$.
  typedef Trajectory<StampedVector3d> TrajectoryV3d;
  /// \brief Trajectory in \f$R^n\f$.
  typedef Trajectory<StampedVectorNd> TrajectoryNd;

  /// \brief Display a trajectory.
  ///
  /// \param stream Output stream
  /// \param trajectory Displayed trajectory
  template <typename T>
  std::ostream& operator<<(std::ostream& stream,
			   const Trajectory<T>& trajectory);

} // end of namespace walk.

# include <walk_interfaces/trajectory.hxx>
#endif //! WALK_INTERFACE_STAMPED_TRAJECTORY_HH
