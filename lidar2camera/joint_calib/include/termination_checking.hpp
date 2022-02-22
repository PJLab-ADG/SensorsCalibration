#ifndef TERMINATION_CHECKING_HPP_
#define TERMINATION_CHECKING_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

#include "params.hpp"
// Termination checking callback. This is needed to finish the
// optimization when an absolute error threshold is met, as opposed
// to Ceres's function_tolerance, which provides for finishing when
// successful steps reduce the cost function by a fractional amount.
// In this case, the callback checks for the absolute average reprojection
// error and terminates when it's below a threshold (for example all
// points < 0.5px error).
class TerminationCheckingCallback : public ceres::IterationCallback {
public:
  TerminationCheckingCallback(const Eigen::MatrixXd &x1,
                              const Eigen::MatrixXd &x2,
                              const EstimateHomographyOptions &options,
                              Eigen::Matrix3d *H)
      : options_(options), x1_(x1), x2_(x2), H_(H) {}

  virtual ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary) {
    // If the step wasn't successful, there's nothing to do.
    if (!summary.step_is_successful) {
      return ceres::SOLVER_CONTINUE;
    }

    // Calculate average of symmetric geometric distance.
    double average_distance = 0.0;
    for (int i = 0; i < x1_.cols(); i++) {
      average_distance +=
          Utils::SymmetricGeometricDistance(*H_, x1_.col(i), x2_.col(i));
    }
    average_distance /= x1_.cols();

    if (average_distance <= options_.expected_average_symmetric_distance) {
      return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    }
    return ceres::SOLVER_CONTINUE;
  }

private:
  const EstimateHomographyOptions &options_;
  const Eigen::MatrixXd &x1_;
  const Eigen::MatrixXd &x2_;
  Eigen::Matrix3d *H_;
};

#endif // TERMINATION_CHECKING_HPP_
