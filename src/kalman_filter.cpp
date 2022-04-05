#include "ros2_imu_filter/kalman_filter.h"

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> kalman_filter(
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

// int main()
// {

//     MatrixXf x(2, 1);// Initial state (location and velocity) 
//     x << 0,
//     	    0; 
//     MatrixXf P(2, 2);//Initial Uncertainty
//     P << 100, 0, 
//     	    0, 100; 
//     MatrixXf u(2, 1);// External Motion
//     u << 0,
//     	    0; 
//     MatrixXf F(2, 2);//Next State Function
//     F << 1, 1,
//     	    0, 1; 
//     MatrixXf H(1, 2);//Measurement Function
//     H << 1,
//     	    0; 
//     MatrixXf R(1, 1); //Measurement Uncertainty
//     R << 1;
//     MatrixXf I(2, 2);// Identity Matrix
//     I << 1, 0,
//     	    0, 1; 

//     tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
//     cout << "x= " << x << endl;
//     cout << "P= " << P << endl;

//     return 0;
// }
