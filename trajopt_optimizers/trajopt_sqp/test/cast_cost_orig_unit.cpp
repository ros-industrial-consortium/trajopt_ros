#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_scene_graph/utils.h>
#include <ifopt/problem.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

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

class CastTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Plotter */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.srdf");
    auto tmp = TRAJOPT_DIR;
    std::cout << tmp;

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;
  }
};

TEST_F(CastTest, boxes)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastTest, boxes");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  //  plotter_->plotScene();

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  collisions.clear();

  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0, 1.9;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_1");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    positions.push_back(pos);
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_2");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // Step 3: Setup collision
  auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  double margin_coeff = 20;
  double margin = 0.3;
  trajopt::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_margin_buffer = 0.05;



  // 4) Add constraints
  { // Fix start position
    std::vector<trajopt::JointPosition::ConstPtr> fixed_vars = {vars[0]};
    auto cnt = std::make_shared<trajopt::JointPosConstraint>(positions[0], fixed_vars);
    nlp.AddConstraintSet(cnt);
  }

  { // Fix end position
    std::vector<trajopt::JointPosition::ConstPtr> fixed_vars = {vars[2]};
    auto cnt = std::make_shared<trajopt::JointPosConstraint>(positions[2], fixed_vars);
    nlp.AddConstraintSet(cnt);
  }

  for (std::size_t i = 1; i < vars.size(); ++i)
  {
    trajopt::ContinuousCollisionEvaluator::Ptr collision_evaluator;
    if (i == 1)
    {
      collision_evaluator = std::make_shared<trajopt::LVSContinuousCollisionEvaluator>(
          kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_config, ContinuousCollisionEvaluatorType::START_FIXED_END_FREE);
    }
    else
    {
      collision_evaluator = std::make_shared<trajopt::LVSContinuousCollisionEvaluator>(
          kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_config, ContinuousCollisionEvaluatorType::START_FREE_END_FIXED);
    }

    auto cnt = std::make_shared<trajopt::ContinuousCollisionConstraintIfopt>(collision_evaluator, GradientCombineMethod::WEIGHTED_LEAST_SQUARES, vars[i-1], vars[i]);
    nlp.AddConstraintSet(cnt);
  }

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;

  // 5) choose solver and options
//  ifopt::IpoptSolver ipopt;
//  ipopt.SetOption("derivative_test", "first-order");
//  ipopt.SetOption("linear_solver", "mumps");
//  //   ipopt.SetOption("jacobian_approximation", "finite-difference-values");
//  ipopt.SetOption("jacobian_approximation", "exact");
//  ipopt.SetOption("print_level", 5);
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(true);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // solve
  solver.verbose = true;
  solver.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();


  // 6) solve
//  ipopt.Solve(nlp);
//  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.transpose() << std::endl;

//  EXPECT_TRUE(ipopt.GetReturnStatus() == 0);

  tesseract_common::TrajArray inputs(3, 2);
  inputs << -1.9, 0, 0, 1.9, 1.9, 3.8;
  Eigen::Map<tesseract_common::TrajArray> results(x.data(), 3, 2);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found =
      checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), inputs, config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), results, config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
