#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <Eigen/Geometry>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/typedefs.hpp>

namespace trajopt
{
/**
Extract trajectory array from solution vector x using indices in array vars
*/
TrajArray getTraj(const DblVec& x, const VarArray& vars);
TrajArray getTraj(const DblVec& x, const AffArray& arr);

inline DblVec trajToDblVec(const TrajArray& x) { return DblVec(x.data(), x.data() + x.rows() * x.cols()); }
inline Eigen::VectorXd concat(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
  Eigen::VectorXd out(a.size() + b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
std::vector<T> concat(const std::vector<T>& a, const std::vector<T>& b)
{
  std::vector<T> out;
  std::vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

template <typename T>
std::vector<T> singleton(const T& x)
{
  return std::vector<T>(1, x);
}

void AddVarArrays(sco::OptProb& prob,
                  int rows,
                  const std::vector<int>& cols,
                  const std::vector<std::string>& name_prefix,
                  const std::vector<VarArray*>& newvars);

void AddVarArray(sco::OptProb& prob, int rows, int cols, const std::string& name_prefix, VarArray& newvars);

/** @brief Store Safety Margin Data for a given timestep */
struct SafetyMarginData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SafetyMarginData>;
  using ConstPtr = std::shared_ptr<const SafetyMarginData>;

  SafetyMarginData(const double& default_safety_margin, const double& default_safety_margin_coeff)
    : default_safety_margin_data_(default_safety_margin, default_safety_margin_coeff)
    , max_safety_margin_(default_safety_margin)
  {
  }

  /**
   * @brief Set the safety margin for a given contact pair
   *
   * The order of the object names does not matter, that is handled internal to
   * the class.
   *
   * @param obj1 The first object name
   * @param obj2 The Second object name
   * @param safety_margins contacts with distance < safety_margin are penalized
   * @param safety_margin_coeffs A safety margin coefficient vector where each
   * element corresponds to a given timestep.
   */
  void setPairSafetyMarginData(const std::string& obj1,
                               const std::string& obj2,
                               const double& safety_margin,
                               const double& safety_margin_coeff)
  {
    Eigen::Vector2d data(safety_margin, safety_margin_coeff);

    pair_lookup_table_[obj1 + obj2] = data;
    pair_lookup_table_[obj2 + obj1] = data;

    if (safety_margin > max_safety_margin_)
    {
      max_safety_margin_ = safety_margin;
    }
  }

  /**
   * @brief Get the pairs safety margin data
   *
   * If a safety margin for the request pair does not exist it returns the default safety margin data.
   *
   * @param obj1 The first object name
   * @param obj2 The second object name
   * @return A Vector2d[Contact Distance Threshold, Coefficient]
   */
  const Eigen::Vector2d& getPairSafetyMarginData(const std::string& obj1, const std::string& obj2) const
  {
    const std::string& key = obj1 + obj2;
    auto it = pair_lookup_table_.find(key);

    if (it != pair_lookup_table_.end())
    {
      return it->second;
    }

    return default_safety_margin_data_;
  }

  /**
   * @brief Get the max safety margin data
   *
   * This used when setting the contact distance in the contact manager.
   *
   * @return Max contact distance threshold
   */
  const double& getMaxSafetyMargin() const { return max_safety_margin_; }

private:
  /// The coeff used during optimization
  /// safety margin: contacts with distance < dist_pen are penalized
  /// Stores [dist_pen, coeff]
  Eigen::Vector2d default_safety_margin_data_;

  /// This use when requesting collision data because you can only provide a
  /// single contact distance threshold.
  double max_safety_margin_;

  /// A map of link pair to contact distance setting [dist_pen, coeff]
  AlignedUnorderedMap<std::string, Eigen::Vector2d> pair_lookup_table_;
};

/**
 * @brief This is a utility function for creating the Safety Margin data vector
 * @param num_elements The number of objects to create
 * @param default_safety_margin Default safety margin
 * @param default_safety_margin_coeff Default safety margin coeff
 * @return A vector of Safety Margin Data
 */
inline std::vector<SafetyMarginData::Ptr> createSafetyMarginDataVector(int num_elements,
                                                                       const double& default_safety_margin,
                                                                       const double& default_safety_margin_coeff)
{
  std::vector<SafetyMarginData::Ptr> info;
  info.reserve(static_cast<size_t>(num_elements));
  for (auto i = 0; i < num_elements; ++i)
  {
    info.push_back(std::make_shared<SafetyMarginData>(default_safety_margin, default_safety_margin_coeff));
  }
  return info;
}

/**
 * @brief Calculate the rotation error vector given a rotation error matrix where the angle is between [-pi, pi]
 * @details This should be used only for calculating the error. Do not use for numerically calculating jacobians
 * because it breaks down a -PI and PI
 * @param R rotation error matrix
 * @return Rotation error vector = Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()
 */
inline Eigen::Vector3d calcRotationalError(const Eigen::Ref<const Eigen::Matrix3d>& R)
{
  Eigen::Quaterniond q(R);
  Eigen::AngleAxisd r12(q);

  // Eigen angle axis flips the sign of axis so rotation is always positive which is
  // not ideal for numerical differentiation.
  int s = (q.vec().dot(r12.axis()) < 0) ? -1 : 1;

  // Make sure that the angle is on [-pi, pi]
  const static double two_pi = 2.0 * M_PI;
  double angle = s * r12.angle();
  Eigen::Vector3d axis = s * r12.axis();
  angle = copysign(fmod(fabs(angle), two_pi), angle);
  if (angle < -M_PI)
    angle += two_pi;
  else if (angle > M_PI)
    angle -= two_pi;

  assert(std::abs(angle) <= M_PI);

  return axis * angle;
}

/**
 * @brief Calculate the rotation error vector given a rotation error matrix where the angle is between [0, 2 * pi]
 * @details This should be used numerically calculating rotation jacobians
 * @param R rotation error matrix
 * @return Rotation error vector = Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()
 */
inline Eigen::Vector3d calcRotationalError2(const Eigen::Ref<const Eigen::Matrix3d>& R)
{
  Eigen::Quaterniond q(R);
  Eigen::AngleAxisd r12(q);

  // Eigen angle axis flips the sign of axis so rotation is always positive which is
  // not ideal for numerical differentiation.
  int s = (q.vec().dot(r12.axis()) < 0) ? -1 : 1;

  // Make sure that the angle is on [0, 2 * pi]
  const static double two_pi = 2.0 * M_PI;
  double angle = s * r12.angle();
  Eigen::Vector3d axis = s * r12.axis();
  angle = copysign(fmod(fabs(angle), two_pi), angle);
  if (angle < 0)
    angle += two_pi;
  else if (angle > two_pi)
    angle -= two_pi;

  assert(angle <= two_pi && angle >= 0);

  return axis * angle;
}

/**
 * @brief Calculate error between two transfroms expressed in t1 coordinate system
 * @param t1 Target Transform
 * @param t2 Current Transform
 * @return error [Position, Rotational(Angle Axis)]
 */
inline Eigen::VectorXd calcTransformError(const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2)
{
  Eigen::Isometry3d pose_err = t1.inverse() * t2;
  return concat(pose_err.translation(), calcRotationalError(pose_err.rotation()));
}

/**
 * @brief Apply a twist for dt to a given transform
 * @param t1 The transform to apply twist.
 * @param twist The twist to apply.
 * @param dt The delta time for which the twist is applied
 * @return Transform result of applying a twist for dt.
 */
inline Eigen::Isometry3d addTwist(const Eigen::Isometry3d& t1,
                                  const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist,
                                  double dt)
{
  Eigen::Isometry3d t2;
  t2.setIdentity();
  Eigen::Vector3d angle_axis = (t1.rotation().inverse() * twist.tail(3)) * dt;
  t2.linear() = t1.rotation() * Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
  t2.translation() = t1.translation() + twist.head(3) * dt;
  return t2;
}

/**
 * @brief Determines if the input superset includes the input subset
 * @param subset
 * @param superset
 * @return
 */
bool isSuperset(const std::vector<std::string>& subset, const std::vector<std::string>& superset);

/**
 * @brief Updates a superset of joint values with those from a subset of joint values
 * @param superset_names: Vector of names of the superset variables
 * @param superset_vals: Values of the superset
 * @param subset_names: Vector of names of the subset variables
 * @param subset_vals: Values of the subset
 * @param new_superset_vals: [output] the updated superset values
 * @return True on success, false on failure
 */
bool updateFromSubset(const std::vector<std::string>& superset_names,
                      const Eigen::VectorXd& superset_vals,
                      const std::vector<std::string>& subset_names,
                      const Eigen::VectorXd& subset_vals,
                      Eigen::Ref<Eigen::VectorXd> new_superset_vals);

/**
 * @brief Gets the subset of joint values from a superset of joint values
 * @param superset_names: Vector of superset variable names
 * @param superset_vals: Vector of superset variable values
 * @param subset_names: Vector of subset variable names
 * @param subset_vals [output]: Subset variable values
 * @return True on success, false on failure
 */
bool getSubset(const std::vector<std::string>& superset_names,
               const Eigen::VectorXd& superset_vals,
               const std::vector<std::string>& subset_names,
               Eigen::Ref<Eigen::VectorXd> subset_vals);

}  // namespace trajopt
