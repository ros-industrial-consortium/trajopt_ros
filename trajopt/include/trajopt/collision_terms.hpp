#pragma once
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <trajopt/cache.hxx>
#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>

namespace trajopt
{
struct CollisionEvaluator
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CollisionEvaluator>;

  CollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                     tesseract_environment::Environment::ConstPtr env,
                     tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                     Eigen::Isometry3d world_to_base,
                     SafetyMarginData::ConstPtr safety_margin_data)
    : manip_(manip)
    , env_(env)
    , adjacency_map_(adjacency_map)
    , world_to_base_(world_to_base)
    , safety_margin_data_(safety_margin_data)
  {
  }
  virtual ~CollisionEvaluator() = default;
  virtual void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) = 0;
  void GetCollisionsCached(const DblVec& x, tesseract_collision::ContactResultVector&);
  virtual void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) = 0;
  virtual sco::VarVector GetVars() = 0;

  const SafetyMarginData::ConstPtr getSafetyMarginData() const { return safety_margin_data_; }
  Cache<size_t, tesseract_collision::ContactResultVector, 10> m_cache;

protected:
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_environment::AdjacencyMap::ConstPtr adjacency_map_;
  Eigen::Isometry3d world_to_base_;
  SafetyMarginData::ConstPtr safety_margin_data_;

private:
  CollisionEvaluator() {}
};

struct SingleTimestepCollisionEvaluator : public CollisionEvaluator
{
public:
  SingleTimestepCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                                   tesseract_environment::Environment::ConstPtr env,
                                   tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                                   Eigen::Isometry3d world_to_base,
                                   SafetyMarginData::ConstPtr safety_margin_data,
                                   const sco::VarVector& vars);
  /**
  @brief linearize all contact distances in terms of robot dofs
  ;
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance
  function
  */
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return m_vars; }

private:
  sco::VarVector m_vars;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

struct CastCollisionEvaluator : public CollisionEvaluator
{
public:
  CastCollisionEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                         tesseract_environment::Environment::ConstPtr env,
                         tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                         Eigen::Isometry3d world_to_base,
                         SafetyMarginData::ConstPtr safety_margin_data,
                         const sco::VarVector& vars0,
                         const sco::VarVector& vars1);
  void CalcDistExpressions(const DblVec& x, sco::AffExprVector& exprs) override;
  void CalcDists(const DblVec& x, DblVec& exprs) override;
  void CalcCollisions(const DblVec& x, tesseract_collision::ContactResultVector& dist_results) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector GetVars() override { return concat(m_vars0, m_vars1); }

private:
  sco::VarVector m_vars0;
  sco::VarVector m_vars1;
  tesseract_collision::ContinuousContactManager::Ptr contact_manager_;
};

class TRAJOPT_API CollisionCost : public sco::Cost, public Plotter
{
public:
  /* constructor for single timestep */
  CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                tesseract_environment::Environment::ConstPtr env,
                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                Eigen::Isometry3d world_to_base,
                SafetyMarginData::ConstPtr safety_margin_data,
                const sco::VarVector& vars);
  /* constructor for cast cost */
  CollisionCost(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                tesseract_environment::Environment::ConstPtr env,
                tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                Eigen::Isometry3d world_to_base,
                SafetyMarginData::ConstPtr safety_margin_data,
                const sco::VarVector& vars0,
                const sco::VarVector& vars1);
  virtual sco::ConvexObjective::Ptr convex(const DblVec& x, sco::Model* model) override;
  virtual double value(const DblVec&) override;
  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override;
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};

class TRAJOPT_API CollisionConstraint : public sco::IneqConstraint
{
public:
  /* constructor for single timestep */
  CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                      tesseract_environment::Environment::ConstPtr env,
                      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                      Eigen::Isometry3d world_to_base,
                      SafetyMarginData::ConstPtr safety_margin_data,
                      const sco::VarVector& vars);
  /* constructor for cast cost */
  CollisionConstraint(tesseract_kinematics::ForwardKinematics::ConstPtr manip,
                      tesseract_environment::Environment::ConstPtr env,
                      tesseract_environment::AdjacencyMap::ConstPtr adjacency_map,
                      Eigen::Isometry3d world_to_base,
                      SafetyMarginData::ConstPtr safety_margin_data,
                      const sco::VarVector& vars0,
                      const sco::VarVector& vars1);
  virtual sco::ConvexConstraints::Ptr convex(const DblVec& x, sco::Model* model) override;
  virtual DblVec value(const DblVec&) override;
  void Plot(const DblVec& x);
  sco::VarVector getVars() override { return m_calc->GetVars(); }

private:
  CollisionEvaluator::Ptr m_calc;
};
}  // namespace trajopt
