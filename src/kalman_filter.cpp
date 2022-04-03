#include "ros2_imu_filter/kalman_filter.h"

// float measurements[3] = { 1, 2, 3 };

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> kalman_filter(
    Eigen::MatrixXf x, Eigen::MatrixXf P, Eigen::MatrixXf u, Eigen::MatrixXf F, 
    Eigen::MatrixXf H, Eigen::MatrixXf R, Eigen::MatrixXf I, uint32_t measurements)
{
    for (uint32_t n = 0; n < measurements; n++) {
        // Measurement Update
        Eigen::MatrixXf Z(1, 1);
        //Z << measurements[n];

        Eigen::MatrixXf y(1, 1);
        y << Z - (H * x);

        Eigen::MatrixXf S(1, 1);
        S << H * P * H.transpose() + R;

        Eigen::MatrixXf K(2, 1);
        K << P * H.transpose() * S.inverse();

        x << x + (K * y);

        P << (I - (K * H)) * P;

        // Prediction
        x << (F * x) + u;
        P << F * P * F.transpose();
    }

    return std::make_tuple(x, P);
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
