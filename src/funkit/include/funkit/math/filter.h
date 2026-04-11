#pragma once

// #include <frc/EigenCore.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

namespace funkit::math {

using mat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

/**
 * LinearKalmanFilter
 * 
 * A templated class which reduces the noise in dynamic systems using linear algebra to accurately find values.
 * @tparam N - the number of variables in the dynamic system
 */
template <int N> class LinearKalmanFilter {
public:
  /**
   * LinearKalmanFilter()
   * 
   * The constructor for the LinearKalmanFilter. 
   * Initially assumes the covariance (uncertainty) to be within one unit of the estimate.
   * @param starterState - The initial state of the dynamic system
   * @param transformerMatrix - A matrix which transforms your state into its updated values.
   */
  LinearKalmanFilter(mat starterState, mat transformerMatrix) {
    pTransformer = transformerMatrix;
    state = starterState;
    covar = Eigen::MatrixXd::Identity(N, N);
  }

  LinearKalmanFilter() {}

  /**
   * Predict()
   * 
   * Predicts the next state and covariance of the system
   * @param pCovariance - The uncertainty caused from noise.
   */
  void Predict(mat pCovariance) {
    state = pTransformer * state;
    covar = pTransformer * covar * pTransformer.transpose() + pCovariance;
  }

  /**
   * Update()
   * 
   * Updates the covariance of the dynamic system
   * @param H - 
   * @param z - 
   * @param var - 
   */
  void Update(mat H, mat z, mat var) {
    mat upCov = var.asDiagonal();
    mat pre_fit_resid = z - H * state;
    mat pre_fit_cov = H * covar * H.transpose() + upCov;
    mat kalm_gain = covar * H.transpose() * pre_fit_cov.inverse();
    state = state + kalm_gain * pre_fit_resid;
    mat bel_factor = kalm_gain * H;
    covar = (Eigen::MatrixXd::Identity(bel_factor.rows(), bel_factor.cols()) -
                bel_factor) *
            covar;
  }

  /**
   * getEstimate() 
   * 
   * @return the estimated state
   */
  mat getEstimate() { return state; }

  /**
   * setEstimate()
   * 
   * @param setState - the state to be set to
   * @param var - the variance to be set to
   */
  void setEstimate(mat setState, mat var) {
    state = setState;
    covar = var.asDiagonal();
  }

  /**
   * setSureEstimate()
   * 
   * Sets a highly confident estimated state
   */
  void setSureEstimate(mat setState) {
    state = setState;
    covar = Eigen::MatrixXd::Identity(N, N);
  }

  /**
   * setState()
   * 
   * @param setState - Set the state of the dynamic system.
   */
  void setState(mat setState) { state = setState; }

  /**
   * getCoVar()
   * 
   * @return the covariance of the dynamic system
   */
  mat getCoVar() { return covar; };

protected:
  mat state{N, 1};
  mat covar{N, N};
  mat pTransformer{N, N};

  // MatrixXd x;
};

};  // namespace funkit::math