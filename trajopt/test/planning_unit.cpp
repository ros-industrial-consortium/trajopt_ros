#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <sstream>
#include <gtest/gtest.h>
#include <tesseract_common/types.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt_utils/clock.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_collision;
using namespace tesseract_kinematics;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;

bool plotting = false; /**< Enable plotting */
static const double LONGEST_VALID_SEGMENT_LENGTH = 0.05;

class PlanningTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */
  void SetUp() override
  {
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env_->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    std::unordered_map<std::string, double> ipos;
    ipos["torso_lift_joint"] = 0.0;
    env_->setState(ipos);

    gLogLevel = util::LevelError;
  }
};

TEST_F(PlanningTest, arm_around_table)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("PlanningTest, arm_around_table");

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/arm_around_table.json");

  std::unordered_map<std::string, double> ipos;
  ipos["torso_lift_joint"] = 0;
  ipos["r_shoulder_pan_joint"] = -1.832;
  ipos["r_shoulder_lift_joint"] = -0.332;
  ipos["r_upper_arm_roll_joint"] = -1.011;
  ipos["r_elbow_flex_joint"] = -1.437;
  ipos["r_forearm_roll_joint"] = -1.1;
  ipos["r_wrist_flex_joint"] = -1.926;
  ipos["r_wrist_roll_joint"] = 3.074;
  env_->setState(ipos);

  //  plotter_->plotScene();

  ProblemConstructionInfo pci(env_);
  pci.fromJson(root);
  pci.basic_info.convex_solver = sco::ModelType::OSQP;
  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = prob->GetEnv()->getStateSolver();
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env_->getSceneGraph(), prob->GetKin()->getActiveLinkNames(), prob->GetEnv()->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
  bool found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), prob->GetInitTraj(), config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  CONSOLE_BRIDGE_logDebug("DOF: %d", prob->GetNumDOF());
  //  if (plotting)
  //  {
  //    opt.addCallback(PlotCallback(*prob, plotter_));
  //  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();
  sco::OptStatus status = opt.optimize();
  EXPECT_TRUE(status == sco::OptStatus::OPT_CONVERGED);
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);

  double d = 0;
  TrajArray traj = getTraj(opt.x(), prob->GetVars());
  for (unsigned i = 1; i < traj.rows(); ++i)
  {
    for (unsigned j = 0; j < traj.cols(); ++j)
    {
      d += std::abs(traj(i, j) - traj(i - 1, j));
    }
  }
  CONSOLE_BRIDGE_logDebug("trajectory norm: %.3f", d);

  //  if (plotting)
  //  {
  //    plotter_->clear();
  //  }

  collisions.clear();
  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()), config);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
