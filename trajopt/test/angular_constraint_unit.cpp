#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <trajopt/problem_description.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/config.hpp>
#include <urdf_parser/urdf_parser.h>
#include <jsoncpp/json/json.h>
#include <srdfdom/model.h>

#include <octomap_ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace trajopt;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot description */

bool plotting_ = false;
int steps_ = 5;

struct testInfo {
  std::string method;
  std::string constraint;

  testInfo(std::string m, std::string c) : method(m), constraint(c) {}
};

class AngularConstraintTest : public testing::TestWithParam<testInfo> {
public:
  ros::NodeHandle nh_;
  urdf::ModelInterfaceSharedPtr urdf_model_;   /**< URDF Model */
  srdf::ModelSharedPtr srdf_model_;            /**< SRDF Model */
  tesseract_ros::KDLEnvPtr env_;            /**< Trajopt Basic Environment */
  tesseract_ros::ROSBasicPlottingPtr plotter_; /**< Trajopt Plotter */
  std::vector<Affine3d> pose_inverses_;
  std::string constraint_type_;
  double tol_;

  AngularConstraintTest() : pose_inverses_(steps_) {}

  TrajOptProbPtr jsonMethod()
  {
    std::string config_file;
    nh_.getParam(constraint_type_ + "_json_file", config_file);

    Json::Value root;
    Json::Reader reader;
    bool parse_success = reader.parse(config_file.c_str(), root);
    if (!parse_success)
    {
      ROS_FATAL("Failed to load trajopt json file from ros parameter");
    }

    ProblemConstructionInfo pci(env_);
    pci.fromJson(root);
    for (int i = 0; i < steps_; i++) {
      Affine3d cur_pose;
      if (constraint_type_ == "confined") {
        std::shared_ptr<ConfinedAxisTermInfo> pose = static_pointer_cast<ConfinedAxisTermInfo>(pci.cnt_infos.at(i));
        cur_pose.translation() = pose->xyz;
        Quaterniond q(pose->wxyz(0), pose->wxyz(1), pose->wxyz(2), pose->wxyz(3));
        cur_pose.linear() = q.matrix();
        pose_inverses_.insert(pose_inverses_.begin() + i, cur_pose.inverse());
      }
      else if (constraint_type_ == "conical") {
        std::shared_ptr<ConicalAxisTermInfo> pose = static_pointer_cast<ConicalAxisTermInfo>(pci.cnt_infos.at(i));
        cur_pose.translation() = pose->xyz;
        Quaterniond q(pose->wxyz(0), pose->wxyz(1), pose->wxyz(2), pose->wxyz(3));
        cur_pose.linear() = q.matrix();
        pose_inverses_.insert(pose_inverses_.begin() + i, cur_pose.inverse());
      }
      else {
        ROS_ERROR("%s is not a valid constraint type. Exiting", constraint_type_.c_str());
        exit(-1);
      }
    }

    return ConstructProblem(pci);
  }

  TrajOptProbPtr cppMethod()
  {
    ProblemConstructionInfo pci(env_);

    // Populate Basic Info
    pci.basic_info.n_steps = steps_;
    pci.basic_info.manip = "manipulator";
    pci.basic_info.start_fixed = false;
  //  pci.basic_info.dofs_fixed

    // Create Kinematic Object
    pci.kin = pci.env->getManipulator(pci.basic_info.manip);

    // Populate Init Info
    Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());

    pci.init_info.type = InitInfo::STATIONARY;
    pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

    // Populate Cost Info
    std::shared_ptr<JointVelCostInfo> jv = std::shared_ptr<JointVelCostInfo>(new JointVelCostInfo);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->name = "joint_vel";
    jv->term_type = TT_COST;
    pci.cost_infos.push_back(jv);

    std::shared_ptr<CollisionCostInfo> collision = std::shared_ptr<CollisionCostInfo>(new CollisionCostInfo);
    collision->name = "collision";
    collision->term_type = TT_COST;
    collision->continuous = false;
    collision->first_step = 0;
    collision->last_step = pci.basic_info.n_steps - 1;
    collision->gap = 1;
    collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);
    pci.cost_infos.push_back(collision);

    // Populate Constraints
    double delta = 0.5/pci.basic_info.n_steps;
    for (auto i = 0; i < pci.basic_info.n_steps; ++i)
    {
      Vector3d xyz(0.5, -0.2 + delta * i, 0.62);
      Vector4d wxyz(0.0, 0.0, 1.0, 0.0);

      Affine3d cur_pose;
      cur_pose.translation() = xyz;
      Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
      cur_pose.linear() = q.matrix();
      pose_inverses_.insert(pose_inverses_.begin() + i, cur_pose.inverse());

      if (constraint_type_ == "confined") {
        std::shared_ptr<ConfinedAxisTermInfo> pose(new ConfinedAxisTermInfo);
        pose->tol = tol_;
        pose->axis = 'y';
        pose->term_type = TT_CNT;
        pose->name = "waypoint_cart_" + std::to_string(i);
        pose->link = "tool0";
        pose->timestep = i;
        pose->xyz = Eigen::Vector3d(0.5, -0.2 + delta * i, 0.62);
        pose->wxyz = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
        pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
        pose->axis_coeff = 10.0;
        pose->confined_coeff = 10.0;
        pci.cnt_infos.push_back(pose);
      }
      else if (constraint_type_ == "conical") {
        std::shared_ptr<ConicalAxisTermInfo> pose(new ConicalAxisTermInfo);
        pose->tol = tol_;
        pose->axis = 'z';
        pose->term_type = TT_CNT;
        pose->name = "waypoint_cart_" + std::to_string(i);
        pose->link = "tool0";
        pose->timestep = i;
        pose->xyz = Eigen::Vector3d(0.5, -0.2 + delta * i, 0.62);
        pose->wxyz = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
        pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
        pose->axis_coeff = 10.0;
        pose->conical_coeff = 10.0;
        pci.cnt_infos.push_back(pose);
      }
      else {
        ROS_ERROR("%s is not a valid constraint type. Exiting", constraint_type_.c_str());
        exit(-1);
      }

    }

    return ConstructProblem(pci);
  }

  virtual void SetUp()
  {

    // Initial setup
    std::string urdf_xml_string, srdf_xml_string;
    nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
    urdf_model_ = urdf::parseURDF(urdf_xml_string);

    srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model_->initString(*urdf_model_, srdf_xml_string);
    env_ = tesseract_ros::KDLEnvPtr(new tesseract_ros::KDLEnv);
    plotter_ = tesseract_ros::ROSBasicPlottingPtr(new tesseract_ros::ROSBasicPlotting(env_));
    assert(env_ != nullptr);

    bool success = env_->init(urdf_model_, srdf_model_);
    assert(success);

    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    double delta = 0.05;
    int length = (1/delta);

    for (int x = 0; x < length; ++x)
      for (int y = 0; y < length; ++y)
        for (int z = 0; z < length; ++z)
          full_cloud.push_back(pcl::PointXYZ(-0.5 + x*delta, -0.5 + y*delta, -0.5 + z*delta));

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(full_cloud, pointcloud_msg);

    octomap::Pointcloud octomap_data;
    octomap::pointCloud2ToOctomap(pointcloud_msg, octomap_data);
    octomap::OcTree* octree = new octomap::OcTree(2*delta);
    octree->insertPointCloud(octomap_data, octomap::point3d(0,0,0));

    AttachableObjectPtr obj(new AttachableObject());
    shapes::OcTree* octomap_world = new shapes::OcTree(std::shared_ptr<const octomap::OcTree>(octree));
    Eigen::Affine3d octomap_pose;

    octomap_pose.setIdentity();
    octomap_pose.translation() = Eigen::Vector3d(1, 0, 0);

    obj->name = "octomap_attached";
    obj->collision.shapes.push_back(shapes::ShapeConstPtr(octomap_world));
    obj->collision.shape_poses.push_back(octomap_pose);
    obj->collision.collision_object_types.push_back(CollisionObjectType::UseShapeType);
    env_->addAttachableObject(obj);

    AttachedBodyInfo attached_body;
    attached_body.object_name = "octomap_attached";
    attached_body.parent_link_name = "base_link";
    attached_body.transform = Eigen::Affine3d::Identity();
    env_->attachBody(attached_body);

    // Set the robot initial state
    std::unordered_map<std::string, double> ipos;
    ipos["joint_a1"] = -0.4;
    ipos["joint_a2"] = 0.2762;
    ipos["joint_a3"] = 0.0;
    ipos["joint_a4"] = -1.3348;
    ipos["joint_a5"] = 0.0;
    ipos["joint_a6"] = 1.4959;
    ipos["joint_a7"] = 0.0;

    env_->setState(ipos);

    plotter_->plotScene();

    // Set Log Level
    gLogLevel = util::LevelError;

  }
};

TEST_P(AngularConstraintTest, AngularConstraint)
{
  // Setup Problem
  TrajOptProbPtr prob;
  testInfo info = GetParam();
  std::string method = info.method;
  constraint_type_ = info.constraint;

  int num_iterations = method == "cpp" ? 5 : 1;
  for (int i = 0; i < num_iterations; i++) {
    tol_  = method == "cpp" ? 2.5 * (i + 1) : 15.0;

    if (method == "cpp") {
      prob = cppMethod();
    }
    else if (method == "json") {
      prob = jsonMethod();
    }
    else {
      ROS_ERROR("Method string invalid. Exiting");
      exit(-1);
    }

    tesseract::ContactResultMap collisions;
    const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
    const std::vector<std::string>& link_names = prob->GetKin()->getLinkNames();

    env_->continuousCollisionCheckTrajectory(joint_names, link_names, prob->GetInitTraj(), collisions);

    tesseract::ContactResultVector collision_vector;
    tesseract::moveContactResultsMapToContactResultsVector(collisions, collision_vector);
    ROS_INFO("Initial trajector number of continuous collisions: %lu\n", collision_vector.size());

    BasicTrustRegionSQP opt(prob);
    if (plotting_)
    {
      opt.addCallback(PlotCallback(*prob, plotter_));
    }

    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    ros::Time tStart = ros::Time::now();
    opt.optimize();
    ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

    if (plotting_)
    {
      plotter_->clear();
    }

    collisions.clear();
    env_->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);
    ROS_INFO("Final trajectory number of continuous collisions: %lu\n", collisions.size());

    auto manip = prob->GetKin();
    auto change_base = env_->getLinkTransform(manip->getBaseLinkName());

    Affine3d tcp;
    tcp.setIdentity();

    TrajArray traj = getTraj(opt.x(), prob->GetVars());
    for (auto j = 0; j < traj.rows(); j++) {
      VectorXd joint_angles(traj.cols());
      for (auto k = 0; k < traj.cols(); k++) {
        joint_angles(k) = traj(j,k);
      }

      Affine3d pose;
      manip->calcFwdKin(pose, change_base, joint_angles);

      Affine3d pose_inv = pose_inverses_.at(j);
      Affine3d err = pose_inv * (pose * tcp);

      if (constraint_type_ == "confined") {
        AngleAxisd aa(err.rotation());
        EXPECT_LE(aa.angle(), tol_ * M_PI/180.0 * (1.0 + 1e-4) );

        Vector3d axis = aa.axis();
        double axis_x = axis(0);
        double axis_y = axis(1);
        double axis_z = axis(2);
        EXPECT_NEAR(0.0, axis_x, 1e-2);
        if (axis_y > 0) {
        EXPECT_NEAR(1.0, axis_y, 1e-4);
        }
        else {
          EXPECT_NEAR(-1.0, axis_y, 1e-2);
        }
        EXPECT_NEAR(0.0, axis_z, 1e-3);

      }
      else {
        Matrix3d orientation(err.rotation());
        double angle = acos(orientation(2,2)) * 180.0/M_PI;
        EXPECT_LE(angle, tol_ * (1 + 1e-4));
      }

      Vector3d pos = err.translation();
      EXPECT_NEAR(0.0, pos(0), 1e-4);
      EXPECT_NEAR(0.0, pos(1), 1e-4);
      EXPECT_NEAR(0.0, pos(2), 1e-4);
    }

  }

}

INSTANTIATE_TEST_CASE_P(Tests, AngularConstraintTest, ::testing::Values(testInfo("json", "confined"), testInfo("json", "conical"),
                        testInfo("cpp", "confined"), testInfo("cpp", "conical")) );

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "angular_constraint_unit");
  ros::NodeHandle pnh("~");

  // Get ROS Parameters
  pnh.param("plotting", plotting_, plotting_);

  return RUN_ALL_TESTS();

}


