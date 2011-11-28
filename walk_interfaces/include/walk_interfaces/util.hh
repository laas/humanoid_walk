#ifndef WALK_INTERFACE_UTIL_HH
# define WALK_INTERFACE_UTIL_HH

# include <cmath>

# include <walk_interfaces/types.hh>

namespace walk
{
  // \brief Convert a 2D \f$SO(2)\f$ matrix transform to a 3D
  // \f$SO(3)\f$ matrix transform.
  //
  // \param src 3x3 \f$SO(2)\f$ matrix transform.
  // \param dst 4x4 \f$SO(3)\f$ matrix transform.
  template <class U, class V>
  void trans2dToTrans3d (U& dst, const V& src)
  {
    assert (dst.cols () == 4
	    && "Incorrect columns number for dest matrix, should be 4.");
    assert (dst.rows () == 4
	    && "Incorrect rows number for dest matrix, should be 4.");
    assert (src.cols () == 3
	    && "Wrong columns number for source matrix, should be 3.");
    assert (src.rows () == 3
	    && "Wrong rows number for source matrix, should be 3.");
    assert (!(U::IsRowMajor ^ V::IsRowMajor)
	    && "Source and dest major type mismatch.");

    dst.setIdentity ();

    // Fill rotation.
    dst(0, 0) = src(0, 0);
    dst(0, 1) = src(0, 1);
    dst(1, 0) = src(1, 0);
    dst(1, 1) = src(1, 1);

    // Fill translation.
    if (U::IsRowMajor)
      {
	dst(0, 3) = src(0, 3);
	dst(1, 3) = src(1, 3);
      }
    else
      {
	dst(3, 0) = src(3, 0);
	dst(3, 1) = src(3, 1);
      }
  }

  // \brief Convert a 2D \f$SO(2)\f$ matrix transform to a 3D
  // \f$SO(3)\f$ transformation.
  //
  // \param srcX translation value along x axis.
  // \param srcY translation value along y axis.
  // \param srcTh rotation angle (radians) around z axis.
  // \param dst 4x4 \f$SO(3)\f$ matrix transform.
  template <class U>
  void trans2dToTrans3d (U& dst,
			 const double& srcX,
			 const double& srcY,
			 const double& srcTh)
  {
    assert (dst.cols () == 4
	    && "Incorrect columns number for dest matrix, should be 4.");
    assert (dst.rows () == 4
	    && "Incorrect rows number for dest matrix, should be 4.");

    dst.setIdentity ();

    // Fill rotation.
    dst(0, 0) = cos (srcTh);
    dst(1, 1) = cos (srcTh);

    if (U::IsRowMajor)
      {
	dst(0, 1) = -sin (srcTh);
	dst(1, 0) = sin (srcTh);
      }
    else
      {
	dst(1, 0) = -sin (srcTh);
	dst(0, 1) = sin (srcTh);
      }

    // Fill translation.
    if (U::IsRowMajor)
      {
	dst(0, 3) = srcX;
	dst(1, 3) = srcY;
      }
    else
      {
	dst(3, 0) = srcX;
	dst(3, 1) = srcY;
      }
  }

  /// \brief Convert a structure of size 3 to a Homogeneous 3d matrix.
  /// \param src input structure.
  /// \param dst output matrix of type Homogeneous3d.
  template <class T>
  void convertVector3dToTrans3d (HomogeneousMatrix3d& dst, const T& src)
  {
    assert (dst.cols () == 4
	    && "Incorrect columns number for dest matrix, should be 4.");
    assert (dst.rows () == 4
	    && "Incorrect rows number for dest matrix, should be 4.");

    dst.setIdentity ();

    // Fill translation.
    if (HomogeneousMatrix3d::IsRowMajor)
      {
	dst(0, 3) = src[0];
	dst(1, 3) = src[1];
	dst(2, 3) = src[2];
      }
    else
      {
	dst(3, 0) = src[0];
	dst(3, 1) = src[1];
	dst(3, 2) = src[2];
      }
  }

  /// \brief Convert a row-major homogeneous 3d matrix to a
  /// HomogeneousMatrix3d type.
  ///
  /// \param src input homogeneous 3d matrix.
  /// \param dst output matrix of type Homogeneous3d.
  template <class T>
  void convertToTrans3d (HomogeneousMatrix3d& dst, const T& src)
  {
    assert (dst.cols () == 4
	    && "Incorrect columns number for dest matrix, should be 4.");
    assert (dst.rows () == 4
	    && "Incorrect rows number for dest matrix, should be 4.");

    dst.setIdentity ();

    // Fill matrix.
    for (unsigned rowId = 0; rowId < 4; ++rowId)
      for (unsigned colId = 0; colId < 3; ++colId)
	if (HomogeneousMatrix3d::IsRowMajor)
	  dst(rowId, colId) = src(rowId, colId);
	else
	  dst(colId, rowId) = src(rowId, colId);
  }

  /// \brief Convert a structure of size 2 to a Vector2d object.
  /// \param src input structure.
  /// \param dst output vector of type Vector2d.
  template <class T>
  void convertToVector2d (Vector2d& dst, const T& src)
  {
    assert (dst.size () == 2
	    && "Incorrect dest vector size mismatch, should be 2.");

    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
  }

  /// \brief Convert a structure of size 3 to a Vector3d object.
  /// \param src input structure.
  /// \param dst output vector of type Vector3d.
  template <class T>
  void convertToVector3d (Vector3d& dst, const T& src)
  {
    assert (dst.size () == 3
	    && "Incorrect dest vector size mismatch, should be 3.");

    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
  }

  /// \brief Convert a structure of any size to a Posture object.
  /// \param src input structure.
  /// \param dst output vector of type Posture.
  template <class T>
  void convertToPosture (Posture& dst, const T& src)
  {
    assert (src.size () == dst.size ()
	    && "Source and dest vectors size mismatch, should be equal.");
    assert (src.size () != 0
	    && "Source vector has a zero size.");

    for (unsigned id; id < src.size (); ++id)
      dst[id] = src [id];
  }

} // end of namespace walk.

#endif //! WALK_INTERFACE_UTIL_HH
