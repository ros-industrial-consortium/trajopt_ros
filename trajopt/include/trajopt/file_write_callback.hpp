#pragma once
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>

namespace trajopt
{
Optimizer::Callback WriteCallback(std::shared_ptr<std::ofstream> file,
                                  TrajOptProbPtr prob);
}
