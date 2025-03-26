/*************************************************************************
	> File Name: quintic_polynomial.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Tue Apr  2 20:50:31 2019
 ************************************************************************/

#ifndef QUINTIC_POLYNOMIAL_H
#define QUINTIC_POLYNOMIAL_H

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <Eigen/Eigen>

namespace cpprobotics{

//class QuinticPolynomial{
//public:
//    // 计算五次多项式系数
//  // current parameter at t=0
//  float xs;
//  float vxs;
//  float axs;
//
//  // parameters at target t=t_j
//  float xe;
//  float vxe;
//  float axe;
//
//
//  // function parameters
//  float a0, a1, a2, a3, a4, a5;
//
//  QuinticPolynomial()=default;
//
//  // polynomial parameters
//  QuinticPolynomial(float xs_, float vxs_, float axs_, float xe_, float vxe_, float axe_, float T): xs(xs_), vxs(vxs_), axs(axs_), xe(xe_), vxe(vxe_), axe(axe_), a0(xs_), a1(vxs_), a2(axs_/2.0){
//    Eigen::Matrix3f A;
//    A << std::pow(T, 3),		std::pow(T, 4),     std::pow(T, 5),
//         3*std::pow(T, 2),  4*std::pow(T, 3),   5*std::pow(T, 4),
//         6*T,								12*std::pow(T, 2),  20*std::pow(T, 3);
//    Eigen::Vector3f B;
//    B<< xe - a0 - a1 * T - a2 * std::pow(T, 2),
//        vxe - a1 - 2 * a2 * T,
//        axe - 2 * a2;
//
//    Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
//    a3 = c_eigen[0];
//    a4 = c_eigen[1];
//    a5 = c_eigen[2];
//  };
//
//  float calc_point(float t) const{
//    return a0 + a1*t + a2*std::pow(t, 2) + a3*std::pow(t, 3) + a4*std::pow(t, 4) + a5*std::pow(t, 5);
//  };
//
//  float calc_first_derivative(float t) const{
//    return a1 + 2*a2*t + 3*a3*std::pow(t, 2) + 4*a4*std::pow(t, 3) + 5*a5*std::pow(t, 4);
//  };
//
//  float calc_second_derivative(float t) const{
//    return 2*a2 + 6*a3*t + 12*a4*std::pow(t, 2) + 20*a5*std::pow(t, 3);
//  };
//
//  float calc_third_derivative(float t) const{
//    return 6*a3 + 24*a4*t + 60*a5*std::pow(t, 2);
//  };
//};

    class QuinticPolynomial {
    public:
        // Polynomial coefficients
        float a0, a1, a2, a3, a4, a5;

        QuinticPolynomial() = default;

        QuinticPolynomial(float xs_, float vxs_, float axs_, float xe_, float vxe_, float axe_, float T)
                : xs(xs_), vxs(vxs_), axs(axs_), xe(xe_), vxe(vxe_), axe(axe_), a0(xs_), a1(vxs_), a2(axs_/2.0) {

            // Calculate the polynomial coefficients
//            Eigen::Matrix3f A;
            Eigen::Matrix<float, 3, 3> A;
            A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
                    3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
                    6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);

            Eigen::Vector3f B;
            B << xe - a0 - a1 * T - a2 * std::pow(T, 2),
                    vxe - a1 - 2 * a2 * T,
                    axe - 2 * a2;

            Eigen::Vector3f c_eigen = A.colPivHouseholderQr().solve(B);
            a3 = c_eigen[0];
            a4 = c_eigen[1];
            a5 = c_eigen[2];
        }

//        float calc_point(float t) const {
//            float t2 = t * t;
//            float t3 = t2 * t;
//            float t4 = t2 * t2;
//            float t5 = t3 * t2;
//            return a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
//        }
        float calc_point(float t) const {
            float t2 = t * t, t3 = t2 * t, t4 = t2 * t2, t5 = t3 * t2;
            return a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
        }

//        float calc_first_derivative(float t) const {
//            float t2 = t * t;
//            float t3 = t2 * t;
//            float t4 = t2 * t2;
//            float t5 = t3 * t2;
//            return a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4;
//        }
//
//        float calc_second_derivative(float t) const {
//            float t2 = t * t;
//            float t3 = t2 * t;
//            float t4 = t2 * t2;
//            float t5 = t3 * t2;
//            return 2 * a2 + 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3;
//        }
//
//        float calc_third_derivative(float t) const {
//            float t2 = t * t;
//            float t3 = t2 * t;
//            float t4 = t2 * t2;
//            float t5 = t3 * t2;
//            return 6 * a3 + 24 * a4 * t + 60 * a5 * t2;
//        }
        float calc_derivative(float t, int order) const {
            float t2 = t * t, t3 = t2 * t, t4 = t2 * t2;
            switch (order) {
                case 1:
                    return a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4;
                case 2:
                    return 2 * a2 + 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3;
                case 3:
                    return 6 * a3 + 24 * a4 * t + 60 * a5 * t2;
                default:
                    return 0.0;
            }
        }

    private:
        // Initial and final conditions
        float xs, vxs, axs;
        float xe, vxe, axe;
    };

}
#endif
