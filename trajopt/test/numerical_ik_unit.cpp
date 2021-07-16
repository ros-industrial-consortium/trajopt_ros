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

class NumericalIKTest : public testing::TestWithParam<const char*>
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

    gLogLevel = util::LevelDebug;
  }
};

TEST_F(NumericalIKTest, numerical_ik1)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("NumericalIKTest, numerical_ik1");

  Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/numerical_ik1.json");

  //  plotter_->plotScene();

  ProblemConstructionInfo pci(env_);
  pci.fromJson(root);
  pci.basic_info.convex_solver = sco::ModelType::OSQP;
  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  //  if (plotting)
  //  {
  //    opt.addCallback(PlotCallback(*prob, plotter_));
  //  }

  CONSOLE_BRIDGE_logDebug("DOF: %d", prob->GetNumDOF());
  opt.initialize(DblVec(static_cast<size_t>(prob->GetNumDOF()), 0));
  double tStart = GetClock();
  CONSOLE_BRIDGE_logDebug("Size: %d", opt.x().size());
  std::stringstream ss;
  ss << toVectorXd(opt.x()).transpose();
  CONSOLE_BRIDGE_logDebug("Initial Vars: %s", ss.str().c_str());
  Eigen::Isometry3d change_base = prob->GetEnv()->getLinkTransform(prob->GetKin()->getBaseLinkName());
  Eigen::Isometry3d initial_pose = prob->GetKin()->calcFwdKin(toVectorXd(opt.x()));
  initial_pose = change_base * initial_pose;

  ss = std::stringstream();
  ss << initial_pose.translation().transpose();
  CONSOLE_BRIDGE_logDebug("Initial Position: %s", ss.str().c_str());
  sco::OptStatus status = opt.optimize();
  CONSOLE_BRIDGE_logDebug("Status: %s", sco::statusToString(status).c_str());
  Eigen::Isometry3d final_pose = prob->GetKin()->calcFwdKin(toVectorXd(opt.x()));
  final_pose = change_base * final_pose;

  Eigen::Isometry3d goal;
  goal.setIdentity();
  goal.translation() << 0.4, 0, 0.8;
  goal.linear() = Eigen::Quaterniond(0, 0, 1, 0).toRotationMatrix();

  for (auto i = 0; i < 4; ++i)
  {
    for (auto j = 0; j < 4; ++j)
    {
      EXPECT_NEAR(goal(i, j), final_pose(i, j), 1e-3);
    }
  }

  ss = std::stringstream();
  ss << final_pose.translation().transpose();
  CONSOLE_BRIDGE_logDebug("Final Position: %s", ss.str().c_str());

  ss = std::stringstream();
  ss << toVectorXd(opt.x()).transpose();
  CONSOLE_BRIDGE_logDebug("Final Vars: ", ss.str().c_str());

  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
