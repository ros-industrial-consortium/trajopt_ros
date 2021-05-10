#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <tesseract/tesseract.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <boost/filesystem/path.hpp>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>

const bool DEBUG = false;

inline std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://trajopt") == 0)
  {
    mod_url.erase(0, strlen("package://trajopt"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TRAJOPT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class CartPositionOptimization : public testing::TestWithParam<const char*>
{
public:
  // 1) Create the problem
  ifopt::Problem nlp_;

  Eigen::VectorXd joint_target_;

  void SetUp() override
  {
    if (DEBUG)
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    else
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    // 1)  Load Robot
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    auto tesseract = std::make_shared<tesseract::Tesseract>();
    tesseract->init(urdf_file, srdf_file, locator);

    // Extract necessary kinematic information
    auto forward_kinematics = tesseract->getFwdKinematicsManager()->getFwdKinematicSolver("right_arm");
    auto world_to_base =
        tesseract->getEnvironment()->getCurrentState()->link_transforms.at(forward_kinematics->getBaseLinkName());
    tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
        tesseract->getEnvironment()->getSceneGraph(),
        forward_kinematics->getActiveLinkNames(),
        tesseract->getEnvironment()->getCurrentState()->link_transforms);
    auto kinematic_info = std::make_shared<trajopt::CartPosKinematicInfo>(
        forward_kinematics, adjacency_map, world_to_base, forward_kinematics->getTipLinkName());

    // Get target position
    Eigen::VectorXd start_pos(forward_kinematics->numJoints());
    start_pos << 0.0, 0, 0, -1.0, 0, -1, -0.00;
    if (DEBUG)
      std::cout << "Joint Limits:\n" << forward_kinematics->getLimits().transpose() << std::endl;

    auto target_pose = Eigen::Isometry3d::Identity();
    joint_target_ = start_pos;
    forward_kinematics->calcFwdKin(target_pose, joint_target_);
    target_pose = world_to_base * target_pose;

    // 3) Add Variables
    std::vector<trajopt::JointPosition::ConstPtr> vars;
    for (int ind = 0; ind < 1; ind++)
    {
      auto zero = Eigen::VectorXd::Zero(7);
      auto var = std::make_shared<trajopt::JointPosition>(
          zero, forward_kinematics->getJointNames(), "Joint_Position_" + std::to_string(ind));
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }

    // 4) Add constraints
    for (const auto& var : vars)
    {
      auto cnt = std::make_shared<trajopt::CartPosConstraint>(target_pose, kinematic_info, var);
      nlp_.AddConstraintSet(cnt);
    }

    if (DEBUG)
    {
      nlp_.PrintCurrent();
      std::cout << "Constraint Jacobian: \n" << nlp_.GetJacobianOfConstraints() << std::endl;
    }
  }
};

template <class T>  // Allows for any solver. The solver has to have a .Solve(ifopt::Problem&) function
inline void runTests(T solver, ifopt::Problem nlp_opt, Eigen::VectorXd joint_targets)
{
  solver.Solve(nlp_opt);
  Eigen::VectorXd x = nlp_opt.GetOptVariables()->GetValues();

  for (Eigen::Index i = 0; i < joint_targets.size(); i++)
    EXPECT_NEAR(x[i], joint_targets[i], 1.1e-3);

  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_opt.PrintCurrent();
  }
}
