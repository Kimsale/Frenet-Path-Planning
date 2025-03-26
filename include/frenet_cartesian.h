/*************************************************************************
	> File Name: visualization.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Sat Mar 30 15:51:30 2019
 ************************************************************************/

#ifndef VISUALIZATION_H
#define VISUALIZATION_H


#include<iostream>
#include<random>
#include<cmath>
#include<cubic_spline.h>
#include <sincos.h>

namespace cpprobotics{


std::pair<float, float> cartesian_to_frenet1D(
        float rs, float rx, float ry, float rtheta, float x, float y) {

    float dx = x - rx;
    float dy = y - ry;

    float cos_theta_r = cos(rtheta);
    float sin_theta_r = sin(rtheta);

    float cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    float d_condition = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd);
    float s_condition = rs;

    return std::make_pair(s_condition, d_condition);
}

std::pair<float, float> frenet_to_cartesian1D(float rs, float rx, float ry, float rtheta,
                                              const std::vector<float>& s_condition, const std::vector<float>& d_condition) {
    if (std::fabs(rs - s_condition[0]) >= 1.0e-6) {
        std::cout << "The reference point s and s_condition[0] don't match" << std::endl;
    }

    float cos_theta_r = cos(rtheta);
    float sin_theta_r = sin(rtheta);

    float x = rx - sin_theta_r * d_condition[0];
    float y = ry + cos_theta_r * d_condition[0];

    return std::make_pair(x, y);
}

float NormalizeAngle(float angle) {
    float a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

std::tuple<float, float, float, float, float, float>
        frenet_to_cartesian3D(float rs, float rx, float ry,float rtheta, float rkappa, float rdkappa,
                              const std::array<float, 3>& s_condition,const std::array<float, 3>& d_condition) {
    if (std::fabs(rs - s_condition[0]) >= 1.0e-6) {
        std::cout << "The reference point s and s_condition[0] don't match" << std::endl;
    }

    float cos_theta_r = cos(rtheta);
    float sin_theta_r = sin(rtheta);

    float x = rx - sin_theta_r * d_condition[0];
    float y = ry + cos_theta_r * d_condition[0];

    float one_minus_kappa_r_d = 1 - rkappa * d_condition[0];
    float tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
    float delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
    float cos_delta_theta = cos(delta_theta);

    float theta = NormalizeAngle(delta_theta + rtheta);
    float kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1];

    float kappa = ((((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                      cos_delta_theta * cos_delta_theta) /
                     (one_minus_kappa_r_d) +
                     rkappa) *
                    cos_delta_theta / (one_minus_kappa_r_d));


    float d_dot = d_condition[1] * s_condition[1];

    float v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_condition[1] * s_condition[1] + d_dot * d_dot);

    float delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * (kappa) - rkappa;
    float a = (s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
                s_condition[1] * s_condition[1] / cos_delta_theta *
                (d_condition[1] * delta_theta_prime - kappa_r_d_prime));

    return std::make_tuple(x, y, v, a, theta, kappa);
}

float degrees(float radian) {
    return std::fmod(radian * 180.0 / M_PI + 360.0, 360.0);
}

}

#endif
