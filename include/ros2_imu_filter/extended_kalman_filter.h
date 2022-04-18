#pragma once

#include <tuple>

#include <Eigen/Eigen>

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> extended_kalman_filter(
    Eigen::MatrixXf x_t_1, Eigen::MatrixXf u_t, Eigen::MatrixXf z_t, Eigen::MatrixXf A_t, Eigen::MatrixXf B_t,  
    Eigen::MatrixXf C_t, Eigen::MatrixXf S_t_1, Eigen::MatrixXf R_t, Eigen::MatrixXf Q_t, Eigen::MatrixXf I);