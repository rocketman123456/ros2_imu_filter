#pragma once

#include <tuple>

#include <Eigen/Eigen>

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> kalman_filter(
    Eigen::MatrixXf x, Eigen::MatrixXf P, Eigen::MatrixXf u, Eigen::MatrixXf F, 
    Eigen::MatrixXf H, Eigen::MatrixXf R, Eigen::MatrixXf I, uint32_t measurements);
