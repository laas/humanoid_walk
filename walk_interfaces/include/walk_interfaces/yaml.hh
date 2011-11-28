#ifndef WALK_INTERFACE_YAML_HH
# define WALK_INTERFACE_YAML_HH
# include <boost/filesystem/path.hpp>
# include <walk_interfaces/pattern-generator.hh>

namespace walk
{
  template <typename T>
  class YamlWriter
  {
  public:
    typedef T patternGenerator_t;

    explicit YamlWriter (const patternGenerator_t& pg);
    ~YamlWriter ();

    void write (const std::string& filename) const;
    void write (boost::filesystem::path& filename) const;
    void write (std::ostream& stream) const;

  protected:
    void writeSteps (std::ostream& stream) const;

    template <typename S>
    void writeStep (std::ostream& stream,
		    const StampedFootstep<S>& step) const;

    template <typename U>
    void writeTrajectory (std::ostream& stream, const U& gamma) const;

    template <typename M>
    void writeMatrix (std::ostream& stream, const M& matrix) const;

  private:
    const patternGenerator_t& patternGenerator_;
  };

} // end of namespace walk.

# include <walk_interfaces/yaml.hxx>
#endif //! WALK_INTERFACE_YAML_HH
