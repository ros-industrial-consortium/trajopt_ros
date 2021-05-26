#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
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

static bool plotting = false; /**< Enable plotting */

class CostsTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Trajopt Basic Environment */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */

  void SetUp() override
  {
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env_->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;
  }
};

/**
 * @brief Tests the equality jointPos cost/constraints.
 *
 * Sets a cost targetting the position of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the position of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the
 cost
 * target
 */
TEST_F(CostsTest, equality_jointPos)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, equality_jointPos");

  const double cnt_targ = 0.0;
  const double cost_targ = -0.1;
  const int steps = 10;
  const double cost_tol = 0.01;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that first step velocity should be zero
  auto jv = std::make_shared<JointPosTermInfo>();
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_pos_single";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  auto jv2 = std::make_shared<JointPosTermInfo>();
  jv2->coeffs = std::vector<double>(7, 10.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_pos_all";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check position constraint is satisfied
  double pos;
  for (auto j = 0; j < output.cols(); ++j)
  {
    pos = output(0, j);
    EXPECT_NEAR(pos, cnt_targ, cnt_tol);
  }
  // Check pos cost is working
  for (auto i = 1; i < output.rows(); ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      pos = output(i, j);
      EXPECT_NEAR(pos, cost_targ, cost_tol);
    }
  }
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointPos constraint
 *
 * Sets a cost targetting the position of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 rest.
 * Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1.
 * Checks to make sure that the constraint is met
 */
TEST_F(CostsTest, inequality_jointPos)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, inequality_cnt_jointPos");

  const double lower_tol = -0.1;
  const double upper_tol = 0.2;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that limits position
  auto jv = std::make_shared<JointPosTermInfo>();
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_pos_limits";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // Joint Velocities also have a cost to some non zero value
  auto jv2 = std::make_shared<JointPosTermInfo>();
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.01);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_pos_targ_1";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  auto jv3 = std::make_shared<JointPosTermInfo>();
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_pos_targ_2";
  jv3->term_type = TT_COST;
  pci.cost_infos.push_back(jv3);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check pos cost is working
  double pos;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      pos = output(i, j);
      EXPECT_TRUE(pos < upper_tol + cnt_tol);
      EXPECT_TRUE(pos > lower_tol - cnt_tol);
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows(); ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      pos = output(i, j);
      EXPECT_TRUE(pos < upper_tol + cnt_tol);
      EXPECT_TRUE(pos > lower_tol - cnt_tol);
    }
  }
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests the equality jointVel cost/constraints.
 *
 * Sets a cost targetting the velocity of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the velocity of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the
 cost
 * target
 */
TEST_F(CostsTest, equality_jointVel)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, equality_jointVel");

  const double cnt_targ = 0.0;
  const double cost_targ = 0.1;
  const int steps = 10;
  const double cost_tol = 0.01;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that first step velocity should be zero
  auto jv = std::make_shared<JointVelTermInfo>();
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_vel_single";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  auto jv2 = std::make_shared<JointVelTermInfo>();
  jv2->coeffs = std::vector<double>(7, 10.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_vel_all";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity constraint is satisfied
  double velocity;
  for (auto j = 0; j < output.cols(); ++j)
  {
    velocity = output(1, j) - output(0, j);
    EXPECT_NEAR(velocity, cnt_targ, cnt_tol);
  }
  // Check velocity cost is working
  for (auto i = 1; i < output.rows() - 1; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      velocity = output(i + 1, j) - output(i, j);
      EXPECT_NEAR(velocity, cost_targ, cost_tol);
    }
  }
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointVel constraint
 *
 * Sets a cost targetting the velocity of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 rest.
 * Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1.
 * Checks to make sure that the constraint is met
 */
TEST_F(CostsTest, inequality_jointVel)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, inequality_cnt_jointVel");

  const double lower_tol = -0.1;
  const double upper_tol = 0.2;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that limits velocity
  auto jv = std::make_shared<JointVelTermInfo>();
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_vel_limits";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // Joint Velocities also have a cost to some non zero value
  auto jv2 = std::make_shared<JointVelTermInfo>();
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.0);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_vel_targ_1";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  auto jv3 = std::make_shared<JointVelTermInfo>();
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_vel_targ_2";
  jv3->term_type = TT_COST;
  pci.cost_infos.push_back(jv3);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity cost is working
  double velocity;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      velocity = output(i + 1, j) - output(i, j);
      EXPECT_TRUE(velocity < upper_tol + cnt_tol);
      EXPECT_TRUE(velocity > lower_tol - cnt_tol);
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows() - 1; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      velocity = output(i + 1, j) - output(i, j);
      EXPECT_TRUE(velocity < upper_tol + cnt_tol);
      EXPECT_TRUE(velocity > lower_tol - cnt_tol);
    }
  }
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests the equality jointVel cost/constraints with time parameterization.
 *
 * Sets a cost targetting the velocity of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the velocity of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the cost
 * target
 */
TEST_F(CostsTest, equality_jointVel_time)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, equality_jointVel_time");

  const double cnt_targ = 0.0;
  const double cost_targ = 0.1;
  const int steps = 10;
  const double cost_tol = 0.01;
  const double cnt_tol = 0.0001;
  const double dt_lower = 0.01234;
  const double dt_upper = 1.5678;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = true;
  pci.basic_info.dt_lower_lim = dt_lower;
  pci.basic_info.dt_upper_lim = dt_upper;

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  pci.init_info.type = InitInfo::STATIONARY;

  // Constraint that first step velocity should be zero
  auto jv = std::make_shared<JointVelTermInfo>();
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->upper_tols = std::vector<double>(7, 0.0);
  jv->lower_tols = std::vector<double>(7, 0.0);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_vel_single";
  jv->term_type = TT_CNT | TT_USE_TIME;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  auto jv2 = std::make_shared<JointVelTermInfo>();
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_vel_all";
  jv2->term_type = TT_COST | TT_USE_TIME;
  pci.cost_infos.push_back(jv2);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity constraint is satisfied
  double velocity;
  for (auto j = 0; j < output.cols() - 1; ++j)
  {
    velocity = (output(1, j) - output(0, j)) * output(0 + 1, output.cols() - 1);
    EXPECT_NEAR(velocity, cnt_targ, cnt_tol);
  }
  // Check velocity cost is working
  for (auto i = 1; i < output.rows() - 1; ++i)
  {
    // dt limit is obeyed
    EXPECT_TRUE(output(i, output.cols() - 1) <= dt_upper);
    EXPECT_TRUE(output(i, output.cols() - 1) >= dt_lower);
    for (auto j = 0; j < output.cols() - 1; ++j)
    {
      velocity = (output(i + 1, j) - output(i, j)) * output(i + 1, output.cols() - 1);
      EXPECT_NEAR(velocity, cost_targ, cost_tol);
    }
  }
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointVel constraint with time parameterization
 *
 * Sets a cost targetting the velocity of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 rest.
 * Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1.
 * Checks to make sure that the constraint is met
 */
TEST_F(CostsTest, inequality_jointVel_time)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, inequality_cnt_jointVel_time");

  const double lower_tol = -0.1;
  const double upper_tol = 0.2;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;
  const double dt_lower = 0.01234;
  const double dt_upper = 3.5678;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = true;
  pci.basic_info.dt_lower_lim = dt_lower;
  pci.basic_info.dt_upper_lim = dt_upper;

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.dt = dt_upper - dt_lower;

  // Constraint that limits velocity
  auto jv = std::make_shared<JointVelTermInfo>();
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_vel_limits";
  jv->term_type = TT_CNT | TT_USE_TIME;
  pci.cnt_infos.push_back(jv);

  // Joint Velocities also have a cost to some non zero value
  auto jv2 = std::make_shared<JointVelTermInfo>();
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.01);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_vel_targ_1";
  jv2->term_type = TT_COST | TT_USE_TIME;
  pci.cost_infos.push_back(jv2);

  auto jv3 = std::make_shared<JointVelTermInfo>();
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_vel_targ_2";
  jv3->term_type = TT_COST | TT_USE_TIME;
  pci.cost_infos.push_back(jv3);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check velocity cost is working
  double velocity;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols() - 1; ++j)
    {
      velocity = (output(i + 1, j) - output(i, j)) * output(i + 1, output.cols() - 1);
      EXPECT_TRUE(velocity < (upper_tol + cnt_tol));
      EXPECT_TRUE(velocity > (lower_tol - cnt_tol));
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows() - 1; ++i)
  {
    for (auto j = 0; j < output.cols() - 1; ++j)
    {
      velocity = (output(i + 1, j) - output(i, j)) * output(i + 1, output.cols() - 1);
      EXPECT_TRUE(velocity < (upper_tol + cnt_tol));
      EXPECT_TRUE(velocity > (lower_tol - cnt_tol));
    }
  }
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests the equality jointAcc cost/constraints.
 *
 * Sets a cost targetting the acceleration of all joints at 0.1 for all timesteps.
 * Sets a conflicting constraint on the velocity of all joints for the first timestep only.
 * Checks to make sure that the constraint is met if the first time step and the other time steps are close to the
 cost
 * target
 */
TEST_F(CostsTest, equality_jointAcc)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, equality_jointAcc");

  const double cnt_targ = 0.0;
  const double cost_targ = 0.1;
  const int steps = 10;
  const double cost_tol = 0.01;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that first step velocity should be zero
  auto jv = std::make_shared<JointAccTermInfo>();
  jv->coeffs = std::vector<double>(7, 10.0);
  jv->targets = std::vector<double>(7.0, cnt_targ);
  jv->first_step = 0;
  jv->last_step = 0;
  jv->name = "joint_acc_single";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // All the rest of the joint velocities have a cost to some non zero value
  auto jv2 = std::make_shared<JointAccTermInfo>();
  jv2->coeffs = std::vector<double>(7, 10.0);
  jv2->targets = std::vector<double>(7.0, cost_targ);
  jv2->first_step = 0;
  jv2->last_step = pci.basic_info.n_steps - 1;
  jv2->name = "joint_acc_all";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check acceleration constraint is satisfied
  double accel;
  for (auto j = 0; j < output.cols(); ++j)
  {
    int i = 0;
    accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
    EXPECT_NEAR(accel, cnt_targ, cnt_tol);
  }
  // Check acceleration cost is working
  for (auto i = 1; i < output.rows() - 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
      EXPECT_NEAR(accel, cost_targ, cost_tol);
    }
  }
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);
}

////////////////////////////////////////////////////////////////////
/**
 * @brief Tests inequality jointAcc constraint
 *
 * Sets a cost targetting the acceleration of all joints at -0.5 for the first half of the timesteps and +0.5 for the
 * rest. Sets a conflicting constraint on the velocity of all joints limiting them to +/- 0.1. Checks to make sure
 that
 * the constraint is met
 */
TEST_F(CostsTest, inequality_jointAcc)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CostsTest, inequality_cnt_jointVel");

  const double lower_tol = -0.1;
  const double upper_tol = 0.2;
  const double cost_targ1 = 0.5;
  const double cost_targ2 = -0.5;

  const int steps = 10;
  const double cnt_tol = 0.0001;

  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps;
  pci.basic_info.manip = "right_arm";

  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = env_->getManipulatorManager()->getFwdKinematicSolver(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getJointNames());
  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Constraint that limits accel
  auto jv = std::make_shared<JointAccTermInfo>();
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->targets = std::vector<double>(7.0, 0);
  jv->lower_tols = std::vector<double>(7.0, lower_tol);
  jv->upper_tols = std::vector<double>(7.0, upper_tol);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_acc_limits";
  jv->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv);

  // Joint accel also have a cost to some non zero value
  auto jv2 = std::make_shared<JointAccTermInfo>();
  jv2->coeffs = std::vector<double>(7, 1.0);
  jv2->targets = std::vector<double>(7.0, cost_targ1);
  jv2->lower_tols = std::vector<double>(7.0, -0.01);
  jv2->upper_tols = std::vector<double>(7.0, 0.01);
  jv2->first_step = 0;
  jv2->last_step = (pci.basic_info.n_steps - 1) / 2;
  jv2->name = "joint_acc_targ_1";
  jv2->term_type = TT_COST;
  pci.cost_infos.push_back(jv2);

  auto jv3 = std::make_shared<JointAccTermInfo>();
  jv3->coeffs = std::vector<double>(7, 1.0);
  jv3->targets = std::vector<double>(7.0, cost_targ2);
  jv3->lower_tols = std::vector<double>(7.0, -0.01);
  jv3->upper_tols = std::vector<double>(7.0, 0.01);
  jv3->first_step = (pci.basic_info.n_steps - 1) / 2 + 1;
  jv3->last_step = pci.basic_info.n_steps - 1;
  jv3->name = "joint_acc_targ_2";
  jv3->term_type = TT_COST;
  pci.cost_infos.push_back(jv3);

  TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting)
  {
    opt.addCallback(PlotCallback(*prob, plotter_));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();

  opt.optimize();
  CONSOLE_BRIDGE_logDebug("planning time: %.3f", GetClock() - tStart);

  TrajArray output = getTraj(opt.x(), prob->GetVars());
  std::cout << "Trajectory: \n" << output << "\n";

  // Check accel cost is working
  double accel;
  for (auto i = 0; i < (output.rows()) / 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
      EXPECT_TRUE(accel < upper_tol + cnt_tol);
      EXPECT_TRUE(accel > lower_tol - cnt_tol);
    }
  }
  for (auto i = (output.rows()) / 2 + 1; i < output.rows() - 2; ++i)
  {
    for (auto j = 0; j < output.cols(); ++j)
    {
      accel = output(i, j) - 2 * output(i + 1, j) + output(i + 2, j);
      EXPECT_TRUE(accel < upper_tol + cnt_tol);
      EXPECT_TRUE(accel > lower_tol - cnt_tol);
    }
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
