#include "ros2_imu_filter/extended_kalman_filter.h"

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> extended_kalman_filter(
    Eigen::MatrixXf x_t_1, Eigen::MatrixXf u_t, Eigen::MatrixXf z_t, Eigen::MatrixXf A_t, Eigen::MatrixXf B_t,  
    Eigen::MatrixXf C_t, Eigen::MatrixXf S_t_1, Eigen::MatrixXf R_t, Eigen::MatrixXf Q_t, Eigen::MatrixXf I)
{
    Eigen::MatrixXf x_t_ = A_t * x_t_1 + B_t * u_t;
    Eigen::MatrixXf S_t_ = A_t * S_t_1 * A_t.transpose() + R_t;

    Eigen::MatrixXf temp = C_t * S_t_ * C_t.transpose() + Q_t;
    Eigen::MatrixXf K_t = S_t_ * C_t * temp.inverse();
    Eigen::MatrixXf x_t = x_t_ + K_t * (z_t - C_t * x_t_);
    Eigen::MatrixXf S_t = (I - K_t * C_t) * S_t_;

    return std::make_tuple(x_t, S_t);
}
