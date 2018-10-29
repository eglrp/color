#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"

struct Constraint
{
	unsigned int iIdx;
	unsigned int jIdx;
	Eigen::Isometry3d zbar_ij;
	double scalar;
};

template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
	const Eigen::Quaternion<T>& quaternion) 
{
		Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
		// We choose the quaternion with positive 'w', i.e., the one with a smaller
		// angle that represents this orientation.
		if (normalized_quaternion.w() < 0.) {
			// Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
			normalized_quaternion.w() *= T(-1.);
			normalized_quaternion.x() *= T(-1.);
			normalized_quaternion.y() *= T(-1.);
			normalized_quaternion.z() *= T(-1.);
		}
		// We convert the normalized_quaternion into a vector along the rotation axis
		// with length of the rotation angle.
		const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
			normalized_quaternion.w());
		const double kCutoffAngle = 1e-7;  // We linearize below this angle.
		const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
		return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
			scale * normalized_quaternion.y(),
			scale * normalized_quaternion.z());
}

class SpaCostFunction {
 public:
  SpaCostFunction(const Constraint pose) : pose_(pose) {}

  // Computes the error between the scan-to-submap alignment 'zbar_ij' and the
  // difference of submap pose 'c_i' and scan pose 'c_j' which are both in an
  // arbitrary common frame.
  template <typename T>
  static std::array<T, 6> ComputeUnscaledError(
      const Eigen::Isometry3d& zbar_ij, const T* const c_i_rotation,
      const T* const c_i_translation, const T* const c_j_rotation,
      const T* const c_j_translation) 
  {
    const Eigen::Quaternion<T> R_i_inverse(c_i_rotation[0], -c_i_rotation[1],
                                           -c_i_rotation[2], -c_i_rotation[3]);

    const Eigen::Matrix<T, 3, 1> delta(c_j_translation[0] - c_i_translation[0],
                                       c_j_translation[1] - c_i_translation[1],
                                       c_j_translation[2] - c_i_translation[2]);
    const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

    const Eigen::Quaternion<T> h_rotation_inverse =
        Eigen::Quaternion<T>(c_j_rotation[0], -c_j_rotation[1],
                             -c_j_rotation[2], -c_j_rotation[3]) *
        Eigen::Quaternion<T>(c_i_rotation[0], c_i_rotation[1], c_i_rotation[2],
                             c_i_rotation[3]);
	Eigen::Quaternion<T> q;
	q = zbar_ij.rotation().cast<T>();
	const Eigen::Matrix<T, 3, 1> angle_axis_difference =
		RotationQuaternionToAngleAxisVector(
		h_rotation_inverse*q);
	  std::array<T, 6> ret = {{T(zbar_ij.translation().x()) - h_translation[0],
		  T(zbar_ij.translation().y()) - h_translation[1],
		  T(zbar_ij.translation().z()) - h_translation[2],
		  angle_axis_difference[0], angle_axis_difference[1],
		  angle_axis_difference[2]}};
    return ret;
  }

  // Computes the error scaled by 'sqrt_Lambda_ij', storing it in 'e'.
  template <typename T>
  static void ComputeScaledError(const Constraint& pose,
                                 const T* const c_i_rotation,
                                 const T* const c_i_translation,
                                 const T* const c_j_rotation,
                                 const T* const c_j_translation, T* const e) {
    std::array<T, 6> e_ij =
        ComputeUnscaledError(pose.zbar_ij, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation);
    (Eigen::Map<Eigen::Matrix<T, 6, 1>>(e)) =
        T(pose.scalar) *
        Eigen::Map<Eigen::Matrix<T, 6, 1>>(e_ij.data());
  }

  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    ComputeScaledError(pose_, c_i_rotation, c_i_translation, c_j_rotation,
                       c_j_translation, e);
    return true;
  }

 private:
  const Constraint pose_;
};
