#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <iostream>
#include <utility>
#include <vector>
#include <array>
#include <string>
#include <Eigen/Eigen>
#include <stdexcept>
#include "cpprobotics_types.h"
#include <arm_neon.h>

namespace cpprobotics{

Vec_d vec_diff(Vec_d input){
    Vec_d output;
    for(unsigned int i=1; i<input.size(); i++){
    output.emplace_back(input[i] - input[i-1]);
    }
    return output;
}

Vec_d cum_sum(const Vec_d& input){
    Vec_d output;
    float temp = 0;
    for(float i : input){
        temp += i;
        output.emplace_back(temp);
    }
    return output;
}

class Spline{
public:
    Vec_d x;
    Vec_d y;
    int nx{};
    Vec_d h;
    Vec_d a;
    Vec_d b;
    Vec_d c;
    Vec_d d;

    Spline()= default;
    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    Spline(const Vec_d& x_, const Vec_d& y_):x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_){
        Eigen::MatrixXf A = calc_A();
        Eigen::VectorXf B = calc_B();
        Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
        float *c_pointer = c_eigen.data();
        //Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
        c.assign(c_pointer, c_pointer+c_eigen.rows());

        for(int i=0; i<nx-1; i++){
            d.emplace_back((c[i+1]-c[i])/(3.0*h[i]));
            b.emplace_back((a[i+1] - a[i])/h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
        }
    };

    float calc(float t){
        if(t<x.front() || t>x.back()){
        throw std::invalid_argument( "received value out of the pre-defined range" );
        }
        int seg_id = bisect(t, 0, nx);
        float dx = t - x[seg_id];
        return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
    };

    float calc_d(float t){
        if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
        }
        int seg_id = bisect(t, 0, nx-1);
        float dx = t - x[seg_id];
        return b[seg_id]  + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
    }

    float calc_dd(float t){
        if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
        }
        int seg_id = bisect(t, 0, nx);
        float dx = t - x[seg_id];
//    std::cout << "i:" << seg_id << std::endl;
//    std::cout << "c[seg_id]:" << c[seg_id] << std::endl;
        return 2 * c[seg_id] + 6 * d[seg_id] * dx;
    }

    // C++ do not have this   Calc third derivative
    float calc_ddd(float t){
        if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
        }
        int seg_id = bisect(t, 0, nx);
        float dx = t - x[seg_id];
        return 6 * d[seg_id];
    }

private:
    Eigen::MatrixXf calc_A(){
        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
        A(0, 0) = 1;
//        for(int i=0; i<nx-1; i++){
//        if (i != nx-2){
//            A(i+1, i+1) = 2 * (h[i] + h[i+1]);
//        }
//        A(i+1, i) = h[i];
//        A(i, i+1) = h[i];
//        }
//        A(0, 1) = 0.0;

        float32x4_t h_values[4];
        for (int i = 0; i < nx - 1; i += 4) {
            for (int j = 0; j < 4; j++) {
                h_values[j] = vdupq_n_f32(h[i + j]);
            }

            float32x4_t h_next_values = vld1q_f32(h.data() + i + 1);
            float32x4_t h_current_values = vld1q_f32(h.data() + i);

            float32x4_t h_sum_values = vaddq_f32(h_values[0], h_next_values);

            float32x4_t h_current_extended = vextq_f32(h_values[0], h_sum_values, 1);
            float32x4_t h_next_extended = vextq_f32(h_sum_values, h_next_values, 1);

            float32x4_t h_sum_double = vmulq_f32(h_current_extended, vdupq_n_f32(2));

            vst1q_f32(A.data() + i + 1, h_sum_double);
            vst1q_f32(A.data() + i + nx, h_sum_double);

            vst1q_f32(A.data() + i, h_current_extended);
            vst1q_f32(A.data() + i + nx + 1, h_next_extended);
        }

        A(nx-1, nx-2) = 0.0;
        A(nx-1, nx-1) = 1.0;
        return A;
    };
    Eigen::VectorXf calc_B(){
        Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);

//        for(int i=0; i<nx-2; i++){
//        B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
//        }

        for (int i = 0; i < nx - 2; i += 4) {
            float32x4_t a_next_values = vld1q_f32(a.data() + i + 2);
            float32x4_t a_current_values = vld1q_f32(a.data() + i + 1);
            float32x4_t a_previous_values = vld1q_f32(a.data() + i);

            float32x4_t h_next_values = vld1q_f32(h.data() + i + 1);
            float32x4_t h_current_values = vld1q_f32(h.data() + i);

            float32x4_t diff_next_values = vsubq_f32(a_next_values, a_current_values);
            float32x4_t diff_current_values = vsubq_f32(a_current_values, a_previous_values);

            float32x4_t h_next_inverse = vrecpeq_f32(h_next_values);
            float32x4_t h_current_inverse = vrecpeq_f32(h_current_values);

            float32x4_t result_next = vmulq_f32(diff_next_values, h_next_inverse);
            float32x4_t result_current = vmulq_f32(diff_current_values, h_current_inverse);

            vst1q_f32(B.data() + i + 1, vsubq_f32(result_next, result_current));
        }

        return B;
    };

    int bisect(float t, int start, int end){
        int mid = (start+end)/2;
        if (t==x[mid] || end-start<=1){
            return mid;
        }else if (t>x[mid]){
            return bisect(t, mid, end);
        }else{
            return bisect(t, start, mid);
        }
    }
};

class Spline2D{
public:
    Spline sx;
    Spline sy;
    Vec_d s;

    Spline2D(const Vec_d& x, const Vec_d& y){
        s = calc_s(x, y);
        sx = Spline(s, x);
        sy = Spline(s, y);
    };

    Poi_d calc_position(float s_t){
        float x = sx.calc(s_t);
        float y = sy.calc(s_t);
        return {{x, y}};
    };

    float calc_curvature(float s_t){
        float dx = sx.calc_d(s_t);
        float ddx = sx.calc_dd(s_t);
        float dy = sy.calc_d(s_t);
        float ddy = sy.calc_dd(s_t);
//      float k = (ddy * dx - ddx * dy) / (pow(dx * dx + dy * dy, 1.5));
        float k = (ddy * dx - ddx * dy) / (std::pow(std::pow(dx, 2) + std::pow(dy, 2), 1.5));
        return k;
//      return (ddy * dx - ddx * dy)/(dx * dx + dy * dy);   // python is different
    };

  // C++ do not have this
    float calc_d_curvature(float s_t){
        float dx = sx.calc_d(s_t);
        float ddx = sx.calc_dd(s_t);
        float dddx = sx.calc_ddd(s_t);
        float dy = sy.calc_d(s_t);
        float ddy = sy.calc_dd(s_t);
        float dddy = sy.calc_ddd(s_t);

        float a = dx * ddy - dy * ddx;
        float b = dx * dddy - dy * dddx;
        float c = dx * ddx + dy * ddy;
        float d = dx * dx + dy * dy;

        return (b * d - 3.0 * a * c) / (d * d * d);
    }

    float calc_yaw(float s_t){
        float dx = sx.calc_d(s_t);
        float dy = sy.calc_d(s_t);
        return std::atan2(dy, dx);
    };


private:
    static Vec_d calc_s(Vec_d x, Vec_d y){
        Vec_d ds;
        Vec_d out_s{0};
        Vec_d dx = vec_diff(std::move(x));
        Vec_d dy = vec_diff(std::move(y));

        for(unsigned int i=0; i<dx.size(); i++){
        ds.emplace_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
        }

        Vec_d cum_ds = cum_sum(ds);
        out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
        return out_s;
    };
};
}
#endif