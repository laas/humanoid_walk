#ifndef WALK_INTERFACE_STAMPED_TRAJECTORY_HH
# define WALK_INTERFACE_STAMPED_TRAJECTORY_HH
# include <iosfwd>
# include <vector>

# include <walk_interfaces/types.hh>
# include <walk_interfaces/stamped-position.hh>

namespace walk
{
  template <typename T>
  class Trajectory
  {
  public:
    typedef T element_t;
    typedef WALK_INTERFACES_EIGEN_STL_VECTOR(T) data_t;

    explicit Trajectory();
    explicit Trajectory(const Trajectory<T>&);
    ~Trajectory();
    Trajectory<T>& operator=(const Trajectory<T>&);
    
    data_t& data();
    const data_t& data() const;

    TimeDuration computeLength() const;
    
  private:
    data_t data_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  typedef Trajectory<StampedPosition3d> Trajectory3d;
  typedef Trajectory<StampedPosition2d> Trajectory2d;
  typedef Trajectory<StampedVector2d> TrajectoryV2d;
  typedef Trajectory<StampedVectorNd> TrajectoryNd;

  template <typename T>
  std::ostream& operator<<(std::ostream&, const Trajectory<T>&);
} // end of namespace walk.

# include <walk_interfaces/trajectory.hxx>
#endif //! WALK_INTERFACE_STAMPED_TRAJECTORY_HH
