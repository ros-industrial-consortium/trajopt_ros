#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

#include <tesseract_environment/core/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_scene_graph/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

static bool plotting = false;

class SimpleCollisionTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env_->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));
  }
};

TEST_F(SimpleCollisionTest, spheres)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("SimpleCollisionTest, spheres");

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/simple_collision_test.json");

  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -0.75;
  ipos["spherebot_y_joint"] = 0.75;
  env_->setState(ipos);

  //  plotter_->plotScene();

  TrajOptProb::Ptr prob = ConstructProblem(root, env_);
  ASSERT_TRUE(!!prob);

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = prob->GetEnv()->getStateSolver();
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env_->getSceneGraph(), prob->GetKin()->getActiveLinkNames(), prob->GetEnv()->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0.2);

  collisions.clear();
  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), prob->GetInitTraj(), config);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logDebug((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
    opt.addCallback(PlotCallback(*prob, plotter_));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  if (plotting)
    plotter_->clear();

  collisions.clear();
  std::cout << getTraj(opt.x(), prob->GetVars()) << std::endl;

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
