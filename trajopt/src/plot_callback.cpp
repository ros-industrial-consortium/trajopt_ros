#include <functional>
#include <set>
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

using namespace util;
using namespace std;
namespace trajopt
{
void PlotCosts(const tesseract::BasicPlottingPtr plotter,
               const std::vector<std::string>& joint_names,
               const vector<CostPtr>& costs,
               const vector<ConstraintPtr>& cnts,
               const VarArray& vars,
               const OptResults& results)
{
  plotter->clear();

  for (const CostPtr& cost : costs)
  {
    if (Plotter* plt = dynamic_cast<Plotter*>(cost.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }

  for (const ConstraintPtr& cnt : cnts)
  {
    if (Plotter* plt = dynamic_cast<Plotter*>(cnt.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }

  plotter->plotTrajectory(joint_names, getTraj(results.x, vars));
  plotter->waitForInput();
}

Optimizer::Callback PlotCallback(TrajOptProb& prob, const tesseract::BasicPlottingPtr plotter)
{
  vector<ConstraintPtr> cnts = prob.getConstraints();
  return std::bind(&PlotCosts,
                   plotter,
                   prob.GetKin()->getJointNames(),
                   std::ref(prob.getCosts()),
                   cnts,
                   std::ref(prob.GetVars()),
                   std::placeholders::_2);
}
}
