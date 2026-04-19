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
   * @param transformerMatrix - A matrix which transforms the state into its updated values.                                                                                                                                                                                                                                                                                                                                                                                                                                                           state into its updated values.
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
   * @brief Updates the state vector and covariance matrix of the dynamic system using sensor data and measurement variances
   * @param H - The measurement matrix, maps the state into useable measurements
   * @param z - The actual readings from the sensor
   * @param var - The measurement variance used to find the measurement noise matrix (R) 
   * 
   * @details 
   * The matrix upCov takes only diagonal values from the measurement variance, giving us R (measurement noise), helping determine how trustworthy the prediction is. 
   * Subtracting H * state from z gives the residual, or measurement error of the sensor compared to the prediction
   * Using the formula S = HP'H^T + R, the next line represents the Innovation Covariance in a kalman filter. 
   * We then use this to find the Kalman Gain (K = P * H^T * S^-1), which determines how trustworthy the measurement is compared to the model prediction. 
   * 
   * Using the Kalman Gain, we then properly update the state (state = state + kalm_gain * pre_fit_resid) 
   * and find the bel_factor, or how much uncertainty was removed from the state estimate
   * 
   * Lastly, we then use the bel_factor to update the covariance to remove accumulated uncertainty
   * @see Linear Kalman Filters
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
   * Sets a highly confident estimated state. As a result, covariance matrix is reset to the identity matrix. 
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
  // Represents the state of a dynamic system. It is used specifically for pose in this codebase. 
  mat state{N, 1};
  /**
   * The Covariance Matrix represents the uncertainty in estimate of the system's state. 
   * Diagonal entries represent the variance of each state component
   * Non diagonal entries represent the correlation between error between state components.
   * 
   *                              ––– EXAMPLE ––– 
   * For a 2x1 state matrix of [position, velocity], the Covariance Matrix would be: 
   *                        [   Var(p)     Covar(p, v) ]
   *                        [ Covar(v, p)    Var(v)    ]
   * If Covar() was positive, then an overestimate in one variable correlates to an overestimate in the other
   * If Covar() was negative, then an overestimate in one variable correlates to an underestimate in the other, or vice versa
   */
  mat covar{N, N};
  /**
   * The pTransformer matrix is a matrix which predicts the next state of a system. 
   *                              ––– EXAMPLE ––– 
   * For a 2x1 state matrix of [position, velocity], the transformer matrix would be:
   *                        [ 1  Δt ]
   *                        [ 0  1  ]
   * As multiplying the state by pTransformer predicts the state matrix to be [position + velocity * Δt, velocity] Δt seconds into the future
   */
  mat pTransformer{N, N};

  // MatrixXd x;
};

};  // namespace funkit::matht