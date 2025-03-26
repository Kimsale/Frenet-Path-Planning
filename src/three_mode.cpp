/*************************************************************************
	> File Name: three_mode.cpp
	> Author: Saier Jin
	> Created Time: Fri Jun  3 11:28:17 2023
 ************************************************************************/

#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>
#include <fstream>
#include <chrono>
#include <iomanip>
#include "cubic_spline.h"
#include "frenet_path.h"
#include "frenet_cartesian.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "sincos.h"


//参数
#define SIM_LOOP 500 //500
#define MAX_SPEED  (50.0 / 3.6)     // 最大速度 [m/s]
#define MAX_ACCEL  20              // 最大加速度 [m/ss]
#define MAX_CURVATURE  1.0          // 最大曲率 [1/m]

#define MIN_ROAD_WIDTH  (-1.8)      // 最小道路宽度 [m]
#define MAX_ROAD_WIDTH  1.8         // 最大道路宽度 [m]

#define D_ROAD_W  0.6               // 道路宽度采样间隔 [m]
#define DT  0.2                     // time tick Delta T [s]

#define ROBOT_RADIUS  2.0           // 机器人半径 [m]

// 速度保持的参数
#define MAXT  5.0                   // 最大预测时间 [m]
#define MINT  4.0                   // 最小预测时间 [m]
#define TARGET_SPEED  (10.0 / 3.6)  // 目标速度（即纵向的速度保持） [m/s]
#define D_T_S  (5.0 / 3.6)          // 目标速度采样间隔 [m/s]
#define N_S_SAMPLE  1               // 目标速度的采样数量


// 最小化jerk的二次方 公式中$k_j, k_t, k_d$ 都是自选  损失函数权重
#define KJ  0.1
#define KT  0.1
#define KD  1.0
#define KLAT  1.0
#define KLON  1.0

// 设置容差值
float tolerance = 1e-6;
// 参数为恒定距离和恒定时间定律
float D0 = 1;
float tau = 1;

using namespace cpprobotics;

// 动态障碍物类
class DynamicObstacle {
public:
    float x;
    float y;
    float width;
    float length;
    float heading;
    float velocity;

    DynamicObstacle(float x, float y, float width, float length,
                    float heading, float velocity)
            : x(x), y(y), width(width), length(length), heading(heading),
              velocity(velocity) {}

    void update(float dt) {
        // 根据速度和时间步长更新障碍物位置
        x += velocity * std::cos(heading) * dt;
        y += velocity * std::sin(heading) * dt;
    }

    std::vector<std::pair<float, float>> get_bounding_box() const {
        // 计算Bounding Box的四个角点坐标
        float cos_heading = std::cos(heading);
        float sin_heading = std::sin(heading);
        float x_corners[4] = {-length / 2, length / 2, length / 2, -length / 2};
        float y_corners[4] = {-width / 2, -width / 2, width / 2, width / 2};
        std::vector<std::pair<float, float>> bbox_corners;
        for (size_t i = 0; i < 4; ++i) {
            float x_rot = x_corners[i] * cos_heading - y_corners[i] * sin_heading;
            float y_rot = x_corners[i] * sin_heading + y_corners[i] * cos_heading;
            float x_world = x_rot + x;
            float y_world = y_rot + y;
            bbox_corners.emplace_back(x_world, y_world);
        }

        return bbox_corners;
    }
};

// 汽车类
class Vehicle {
public:
    float _velocity;
    Spline2D _csp;
    float _s;
    float _l;
    float _w;
    float _h;

    Vehicle(Spline2D  csp, float s, float l, float velocity)
            : _velocity(velocity), _csp(std::move(csp)), _s(s), _l(l), _w(2.0), _h(4.8) {}

    void update(float dt) {
        _s += _velocity * dt;
    }

//    float length() const {
//        return _h;
//    }
//
//    float width() const {
//        return _w;
//    }
//
//    std::pair<float, float> position() {
//        std::array<float, 2> position = _csp.calc_position(_s);
//        return {position[0] - _h / 2.0, position[1] - _w / 2.0};
//    }

    float s() const {
        return _s;
    }

    float s_d() const {
        return _velocity;
    }

    static float s_dd() {
        return 0.0;
    }

//    std::vector<std::array<float, 2>> rect() {
//        if (_s < _csp.s.back() - 0.1f) {
//            std::array<float, 2> position = _csp.calc_position(_s);
//            float heading = _csp.calc_yaw(_s);
//
//            std::array<float, 1> s_condition = {_s};
//            std::array<float, 1> d_condition = {_l};
//            std::vector<float> s_condition_vec(s_condition.begin(), s_condition.end());
//            std::vector<float> d_condition_vec(d_condition.begin(), d_condition.end());
//            std::pair<float, float> cartesian_position = frenet_to_cartesian1D(_s, position[0], position[1], heading,
//                                                                                 s_condition_vec, d_condition_vec);
//
//            float cx = cartesian_position.first;
//            float cy = cartesian_position.second;
//
//            float cos_heading = std::cos(heading);
//            float sin_heading = std::sin(heading);
//
//            auto vertex = [&](float e1, float e2) {
//                return std::array<float, 2>{
//                        static_cast<float>(cx + (e1 * _h * cos_heading + e2 * _w * sin_heading) / 2.0),
//                        static_cast<float>(cy + (e1 * _h * sin_heading - e2 * _w * cos_heading) / 2.0)
//                };
//            };
//
//            std::vector<std::array<float, 2>> vertices;
//            vertices.emplace_back(vertex(-1, -1));
//            vertices.emplace_back(vertex(-1, 1));
//            vertices.emplace_back(vertex(1, 1));
//            vertices.emplace_back(vertex(1, -1));
//            vertices.emplace_back(vertices[0]);
//
//            return vertices;
//        } else {
//            return {};
//        }
//    }
//
//    float yaw()  {
//        float heading = _csp.calc_yaw(_s);
//        return degrees(heading);
//    }
};

float sum_of_power(const Vec_d& value_list){
    float sum = 0;
    for(float item:value_list){
        sum += item*item;
    }
    return sum;
}

// 计算frenet跟车轨迹
Vec_Path calc_following_path(
        float c_d, float c_d_d, float c_d_dd, float s0, float s0_d, float s0_dd, float s1, float s1_d, float s1_dd){
    std::vector<FrenetPath> fp_list;
    for (size_t count = 0; count < static_cast<size_t>((MAX_ROAD_WIDTH-MIN_ROAD_WIDTH)/D_ROAD_W); ++count) {
        float di = MIN_ROAD_WIDTH + static_cast<float>(count) * D_ROAD_W;
        for (size_t count2 = 0; count2 < static_cast<size_t>((5-2)/DT); ++count2) {
            float Ti = 2.0 + static_cast<float>(count2) * DT;
            FrenetPath fp;
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for (size_t count3 = 0; count3 < static_cast<size_t>((Ti+tolerance) / DT); ++count3) {
                float t = static_cast<float>(count3) * DT;
                fp.t.emplace_back(t);
                fp.d.emplace_back(lat_qp.calc_point(t));
//                fp.d_d.emplace_back(lat_qp.calc_first_derivative(t));
//                fp.d_dd.emplace_back(lat_qp.calc_second_derivative(t));
//                fp.d_ddd.emplace_back(lat_qp.calc_third_derivative(t));
                fp.d_d.emplace_back(lat_qp.calc_derivative(t, 1));
                fp.d_dd.emplace_back(lat_qp.calc_derivative(t, 2));
                fp.d_ddd.emplace_back(lat_qp.calc_derivative(t, 3));
            }

            float s = s1 - s0;
            for (float delta_s : {s}) {
                float s_target_dd = s1_dd;
                float s_target_d = s1_d;
                float s_target = s0 + delta_s;

                FrenetPath tfp = fp;
                QuinticPolynomial lon_qp(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti);

                for (float i: fp.t) {
                    tfp.s.emplace_back(lon_qp.calc_point(i));
//                    tfp.s_d.emplace_back(lon_qp.calc_first_derivative(i));
//                    tfp.s_dd.emplace_back(lon_qp.calc_second_derivative(i));
//                    tfp.s_ddd.emplace_back(lon_qp.calc_third_derivative(i));
                    tfp.s_d.emplace_back(lon_qp.calc_derivative(i, 1));
                    tfp.s_dd.emplace_back(lon_qp.calc_derivative(i, 2));
                    tfp.s_ddd.emplace_back(lon_qp.calc_derivative(i, 3));
                }

                float Jp = sum_of_power(tfp.d_ddd);
                float Js = sum_of_power(tfp.s_ddd);
                float ds = std::pow(tfp.s.back() - s0, 2);

                tfp.cd = KJ * Jp + KT * 10.0 * Ti + KD * std::pow(tfp.d.back(), 2);
                tfp.cv = KJ * Js + KT * 10.0 * Ti + KD * ds;
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;

                fp_list.emplace_back(tfp);
            }
        }
    }
    return fp_list;
}

// 计算frenet停车轨迹
Vec_Path calc_stopping_path(
        float c_d, float c_d_d, float c_d_dd, float s0, float s0_d, float s0_dd, float s1, float s1_d, float s1_dd){
    std::vector<FrenetPath> fp_list;
    for (size_t count = 0; count < static_cast<size_t>((MAX_ROAD_WIDTH-MIN_ROAD_WIDTH)/D_ROAD_W); ++count) {
        float di = MIN_ROAD_WIDTH + static_cast<float>(count) * D_ROAD_W;
        for (size_t count2 = 0; count2 < static_cast<size_t>((5-2)/DT); ++count2) {
            float Ti = 2.0 + static_cast<float>(count2) * DT;
            FrenetPath fp;
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for (size_t count3 = 0; count3 < static_cast<size_t>((Ti+tolerance) / DT); ++count3) {
                float t = static_cast<float>(count3) * DT;
                fp.t.emplace_back(t);
                fp.d.emplace_back(lat_qp.calc_point(t));
//                fp.d_d.emplace_back(lat_qp.calc_first_derivative(t));
//                fp.d_dd.emplace_back(lat_qp.calc_second_derivative(t));
//                fp.d_ddd.emplace_back(lat_qp.calc_third_derivative(t));
                fp.d_d.emplace_back(lat_qp.calc_derivative(t, 1));
                fp.d_dd.emplace_back(lat_qp.calc_derivative(t, 2));
                fp.d_ddd.emplace_back(lat_qp.calc_derivative(t, 3));
            }

            float s = s1 - s0;
            for (float delta_s : {s}) {
                float s_target_dd = s1_dd;
                float s_target_d = s1_d;
                float s_target = s0 + delta_s;

                FrenetPath tfp = fp;
                QuinticPolynomial lon_qp(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti);

                for (float i: fp.t) {
                    tfp.s.emplace_back(lon_qp.calc_point(i));
//                    tfp.s_d.emplace_back(lon_qp.calc_first_derivative(i));
//                    tfp.s_dd.emplace_back(lon_qp.calc_second_derivative(i));
//                    tfp.s_ddd.emplace_back(lon_qp.calc_third_derivative(i));
                    tfp.s_d.emplace_back(lon_qp.calc_derivative(i, 1));
                    tfp.s_dd.emplace_back(lon_qp.calc_derivative(i, 2));
                    tfp.s_ddd.emplace_back(lon_qp.calc_derivative(i, 3));
                }

                float Jp = sum_of_power(tfp.d_ddd);
                float Js = sum_of_power(tfp.s_ddd);
                float ds = std::pow(tfp.s.back() - s0, 2);

                tfp.cd = KJ * Jp + KT * Ti + KD * std::pow(tfp.d.back(), 2);
                tfp.cv = KJ * Js + KT * Ti + KD * ds;
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;

                fp_list.emplace_back(tfp);
            }
        }
    }
    return fp_list;
}

// 计算frenet速度保持轨迹
Vec_Path calc_frenet_paths(
        float c_speed, float c_d, float c_d_d, float c_d_dd, float s0){
    std::vector<FrenetPath> fp_list;
    for (size_t count = 0; count < static_cast<size_t>((MAX_ROAD_WIDTH-MIN_ROAD_WIDTH)/D_ROAD_W); ++count) {
        float di = MIN_ROAD_WIDTH + static_cast<float>(count) * D_ROAD_W;
        for (size_t count2 = 0; count2 < static_cast<size_t>((MAXT-MINT)/DT); ++count2) {
            float Ti = MINT + static_cast<float>(count2) * DT;
            FrenetPath fp;
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for (size_t count3 = 0; count3 < static_cast<size_t>((Ti+tolerance) / DT); ++count3) {
//                std::cout<< "count3 < " << static_cast<size_t>((Ti+tolerance) / DT) << std::endl;
                float t = static_cast<float>(count3) * DT;
                fp.t.emplace_back(t);
                fp.d.emplace_back(lat_qp.calc_point(t));
//                fp.d_d.emplace_back(lat_qp.calc_first_derivative(t));
//                fp.d_dd.emplace_back(lat_qp.calc_second_derivative(t));
//                fp.d_ddd.emplace_back(lat_qp.calc_third_derivative(t));
                fp.d_d.emplace_back(lat_qp.calc_derivative(t, 1));
                fp.d_dd.emplace_back(lat_qp.calc_derivative(t, 2));
                fp.d_ddd.emplace_back(lat_qp.calc_derivative(t, 3));
            }
            for (size_t count4 = 0; count4 < static_cast<size_t>(2 * N_S_SAMPLE); ++count4) {
                float tv = TARGET_SPEED - D_T_S * N_S_SAMPLE + static_cast<float>(count4) * D_T_S;
                FrenetPath tfp = fp;
                QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);
                for(float t_:fp.t){
                    tfp.s.emplace_back(lon_qp.calc_point(t_));
                    tfp.s_d.emplace_back(lon_qp.calc_first_derivative(t_));
                    tfp.s_dd.emplace_back(lon_qp.calc_second_derivative(t_));
                    tfp.s_ddd.emplace_back(lon_qp.calc_third_derivative(t_));
                }

                float Jp = sum_of_power(tfp.d_ddd);
                float Js = sum_of_power(tfp.s_ddd);

//                float ds = std::pow(tfp.s_d.back() - s0, 2);
                float ds = std::pow(TARGET_SPEED - tfp.s_d.back(), 2);

                tfp.cd = KJ * Jp + KT * Ti + KD * std::pow(tfp.d.back(), 2);
                tfp.cv = KJ * Js + KT * Ti + KD * ds;
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;

                fp_list.emplace_back(tfp);
            }
        }
    }
    return fp_list;
}

// 求解笛卡尔坐标系下的状态量
Vec_Path calc_global_paths(Vec_Path& fplist, Spline2D csp) {
    for (auto& fp : fplist) {
        for (size_t i = 0; i < fp.s.size(); ++i) {
            Poi_d r = csp.calc_position(fp.s[i]);
            if (std::isnan(r[0]) || std::isnan(r[1])) {
                break;
            }

            float rtheta = csp.calc_yaw(fp.s[i]);
            float rkappa = csp.calc_curvature(fp.s[i]);
            float rdkappa = csp.calc_d_curvature(fp.s[i]);

            std::array<float, 3> s_condition{ fp.s[i], fp.s_d[i], fp.s_dd[i] };

            std::array<float, 3> d_condition{ fp.d[i], fp.d_d[i], fp.d_dd[i] };

            float x, y, v, a, theta, kappa;
            std::tie(x, y, v, a, theta, kappa) = frenet_to_cartesian3D(fp.s[i], r[0], r[1], rtheta, rkappa, rdkappa,
                                                                       s_condition, d_condition);

            fp.x.emplace_back(x);
            fp.y.emplace_back(y);
            fp.v.emplace_back(v);
            fp.a.emplace_back(a);
            fp.yaw.emplace_back(theta);

            fp.c.emplace_back(kappa);
        }
    }
    return fplist;
}

// 碰撞检测
bool check_collision(FrenetPath path, std::vector<DynamicObstacle>& ob) {
    if (ob.empty()) {
        return true;
    }
    for (auto & i : ob) {
        const float ob_x = i.x;
        const float ob_y = i.y;

        bool collision = false;
        for (size_t j = 0; j < path.x.size(); ++j) {
            float dx = path.x[j] - ob_x;
            float dy = path.y[j] - ob_y;
            float d_squared = dx * dx + dy * dy;

            if (d_squared <= ROBOT_RADIUS * ROBOT_RADIUS) {
                collision = true;
                break;
            }
        }

        if (collision) {
            return false;
        }
    }

    return true;
}

//最大速度检查 最大加速度检查 最大曲率检查 碰撞检查
Vec_Path check_paths(Vec_Path& fplist, std::vector<DynamicObstacle>& ob) {
    Vec_Path ok_paths;

    for (const auto & path : fplist) {
        bool is_valid = true;

        // Max speed check
        for (float v : path.s_d) {
            if (v > MAX_SPEED) {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) continue;

        // Max acceleration check
        for (float a : path.s_dd) {
            if (std::abs(a) > MAX_ACCEL) {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) continue;

        // Max curvature check
        for (float c : path.c) {
            if (std::abs(c) > MAX_CURVATURE) {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) continue;

        if (!check_collision(path, ob)) {
            is_valid = false;
        }

        if (is_valid) {
            ok_paths.push_back(path);
        }
    }
    return ok_paths;
}


//计算following轨迹  选出最小的cost对应的轨迹
std::tuple<FrenetPath, Vec_Path> frenet_following_planning(
        const Spline2D& csp, float s0, float s0_d, float s0_dd,
        float c_d, float c_d_d, float c_d_dd,
        float s1, float s1_d, float s1_dd, std::vector<DynamicObstacle>& ob){
    Vec_Path fp_list = calc_following_path(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s1, s1_d, s1_dd);
    fp_list = calc_global_paths(fp_list, csp);
    fp_list = check_paths(fp_list, ob);

    float min_cost = std::numeric_limits<float>::infinity();
    FrenetPath final_path;
    for(const auto& path:fp_list){
        if (min_cost >= path.cf){
            min_cost = path.cf;
            final_path = path;
        }
    }
    return std::make_tuple(final_path, fp_list);
}

//计算stopping轨迹  选出最小的cost对应的轨迹
std::tuple<FrenetPath, Vec_Path> frenet_stopping_planning(
        const Spline2D& csp, float s0, float s0_d, float s0_dd,
        float c_d, float c_d_d, float c_d_dd,
        float s1, float s1_d, float s1_dd, std::vector<DynamicObstacle>& ob){
    Vec_Path fp_list = calc_stopping_path(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s1, s1_d, s1_dd);
    fp_list = calc_global_paths(fp_list, csp);
    fp_list = check_paths(fp_list, ob);

    float min_cost = std::numeric_limits<float>::infinity();
    FrenetPath final_path;
    for(const auto& path:fp_list){
        if (min_cost >= path.cf){
            min_cost = path.cf;
            final_path = path;
        }
    }
    return std::make_tuple(final_path, fp_list);
}

//计算速度保持轨迹  选出最小的cost对应的轨迹
std::tuple<FrenetPath, Vec_Path> frenet_velocity_keeping_planning(
        const Spline2D& csp, float s0, float c_speed,
        float c_d, float c_d_d, float c_d_dd, std::vector<DynamicObstacle>& ob){
    Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    fp_list  = calc_global_paths(fp_list, csp);
    Vec_Path save_paths = check_paths(fp_list, ob);

    float min_cost = std::numeric_limits<float>::max();
    FrenetPath final_path;
    for(const auto& path:save_paths){
        if (min_cost >= path.cf){
            min_cost = path.cf;
            final_path = path;
        }
    }
    return std::make_tuple(final_path, save_paths);
}

// 生成目标航线
std::tuple<Vec_d, Vec_d, Vec_d, Vec_d, Spline2D>
        generate_target_course(Vec_d x, Vec_d y) {
    Spline2D csp(std::move(x),std::move(y));

    Vec_d s;
    for (size_t count = 0; count < static_cast<size_t>(csp.s.back()/0.1); ++count){
        float i_s = static_cast<float>(count) * 0.1;
        s.emplace_back(i_s);
    }

    Vec_d r_x;
    Vec_d r_y;
    Vec_d ryaw;
    Vec_d rcurvature;
    for (float i_s : s) {
        std::array<float, 2> point_ = csp.calc_position(i_s);
        r_x.emplace_back(point_[0]);
        r_y.emplace_back(point_[1]);
        ryaw.emplace_back(csp.calc_yaw(i_s));
        rcurvature.emplace_back(csp.calc_curvature(i_s));
    }
    return std::make_tuple(r_x, r_y, ryaw, rcurvature, csp);
}


// 创建车道边界
std::vector<std::array<float, 2>> create_lane_border(std::vector<Vec_d>& ref_line, float width) {
    Vec_Poi border;

    Vec_d tx, ty, tyaw, tc;
    Spline2D csp(ref_line[0], ref_line[1]);

    std:: tie(tx, ty, tyaw, tc, csp) = generate_target_course(
            {ref_line[0][0], ref_line[1][0], ref_line[2][0], ref_line[3][0], ref_line[4][0]},
            {ref_line[0][1], ref_line[1][1], ref_line[2][1], ref_line[3][1], ref_line[4][1]}
    );

    Vec_d s;
    for (size_t count = 0; count < static_cast<size_t>(csp.s.back()/0.1); ++count){
        float i_s = static_cast<float>(count) * 0.1;
        s.emplace_back(i_s);
    }

    for (size_t i = 0; i < s.size(); ++i) {
        Vec_d s_condition = {s[i]};
        Vec_d d_condition = {width};
        std::pair<float, float> position = frenet_to_cartesian1D(s[i], tx[i], ty[i], tyaw[i], s_condition, d_condition);
        border.push_back({position.first, position.second});
    }

    return border;
}

// 寻找障碍物在参考路径上的最近点
int find_nearest_point_on_path(float x1, float y1, Vec_d tx, Vec_d ty){
    float min_distance = std::numeric_limits<float>::infinity();
    int nearest_index = -1;
    for (int i=0; i < tx.size(); ++i){
        float dx = x1 - tx[i];
        float dy = y1 - ty[i];
        float distance = std::hypot(dx, dy);
        if (distance < min_distance){
            min_distance = distance;
            nearest_index = static_cast<int>(i);
        }
    }
    return nearest_index;
}

// 模式选择
enum class Mode : std::uint8_t {
    VELOCITY_KEEPING = 1,
    STOPPING = 2,
    FOLLOWING = 3
};

constexpr float SWITCH_DISTANCE = 15.0; //跟车阈值
constexpr float STOP_DISTANCE = 10.0;   //停车阈值
std::pair<Mode, float> decision(float s, float s_front,
                                 const std::vector<Vec_d>& ob_s,
                                 const std::vector<Vec_d>& ob_d) {
    // 设定一个初始的停止线位置
    float stop_line_s = 80;

    // 检查每个障碍物的情况
    for (size_t i = 0; i < ob_s.size(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            if (std::abs(ob_d[i][j]) <= 1.8) {
                if (0 <= ob_s[i][j] - s && ob_s[i][j] - s <= 10) {
                    stop_line_s = std::min(stop_line_s, (s + 5));
                }
            }
        }
    }

    if (stop_line_s < 80 or s > stop_line_s-STOP_DISTANCE) {
        return std::make_pair(Mode::STOPPING, stop_line_s);
    }

    if (s_front - s < SWITCH_DISTANCE) {
        return std::make_pair(Mode::FOLLOWING, stop_line_s);
    }

    return std::make_pair(Mode::VELOCITY_KEEPING, stop_line_s);
}

int main() {
    std::cout << __FILE__ << " start!!" << std::endl;

    std::vector<Vec_Poi> center_lines;
    std::vector<Vec_Poi> borders;

    std::vector<Vec_d> center_line = {{0.0, 0.0}, {100.0, 0.0}, {205, 0.0}, {350.0, 0}, {705, 0.0}};

    Vec_d tx;
    Vec_d ty;
    Vec_d tyaw;
    Vec_d tc;

    Spline2D csp(
            {center_line[0][0], center_line[1][0], center_line[2][0], center_line[3][0], center_line[4][0]},
            {center_line[0][1], center_line[1][1], center_line[2][1], center_line[3][1], center_line[4][1]}
    );

    std:: tie(tx, ty, tyaw, tc, csp) = generate_target_course(
            {center_line[0][0], center_line[1][0], center_line[2][0], center_line[3][0], center_line[4][0]},
            {center_line[0][1], center_line[1][1], center_line[2][1], center_line[3][1], center_line[4][1]}
    );

    Vec_d border_l = {-1.7, 1.7, 5.1, 8.5};
    Vec_d center_l = {3.4, 6.8};

    for (float i : border_l) {
        std::vector<Poi_d> border = create_lane_border(center_line, i);
        borders.emplace_back(border);
    }

    for (float i : center_l) {
        std::vector<Poi_d> center = create_lane_border(center_line, i);
        center_lines.emplace_back(center);
    }

    float stop_line_s = std::numeric_limits<float>::infinity();

    // initial state
    float s0 = 0.0;
    float s0_d = 10.0 / 3.6;
    float s0_dd = 0.0;
    float c_speed = s0_d;
    float c_d = 0.03;
    float c_d_d = 0.0;
    float c_d_dd = 0.0;
    float s1 = stop_line_s;
    float s1_d = 0.0;
    float s1_dd = 0.0;

    Vehicle car_1 (csp, 25.0, 0.0, 7.0 / 3.6);
    Vehicle car_2 (csp, 0.0, 3.4, 4.0 / 3.6);
    Vehicle car_3 (csp, 0.0, 6.8, 12.0 / 3.6);
    std::vector<Vehicle> cars;
    cars.emplace_back(car_1);
    cars.emplace_back(car_2);
    cars.emplace_back(car_2);

    Vec_d s;
    for (size_t count = 0; count < static_cast<size_t>(csp.s.back()/0.1); ++count){
        float i_s = static_cast<float>(count) * 0.1;
        s.emplace_back(i_s);
    }

    std::vector<DynamicObstacle> ob = {
            {25.0, 9.0, 1.0, 2.0, M_PI * 5 / 4, 7},
            {20.0, 4.0, 1.0, 2.0, M_PI * 4 / 3, 6},
            {20.0, 6.0, 1.0, 2.0, M_PI * 4 / 3, 0.1}
    };

    std::vector<Vec_d> ob_s(ob.size(), Vec_d(4, 0.0));
    std::vector<Vec_d> ob_d(ob.size(), Vec_d(4, 0.0));

    FrenetPath path;
    Vec_Path all_paths;

    auto start = std::chrono::high_resolution_clock::now();

    // 循环仿真时间内LOOP，做frenet最优规划
    for (int i = 0; i < SIM_LOOP; ++i) {
        for (size_t j = 0; j < ob.size(); ++j) {
            ob[j].update(0.2);
            std::vector<std::pair<float, float>> obstacle_bb = ob[j].get_bounding_box();
            for (size_t u = 0; u < obstacle_bb.size(); ++u) {
                // 找到最近的轨迹点索引
                int nearest_index = find_nearest_point_on_path(obstacle_bb[u].first, obstacle_bb[u].second, tx,
                                                               ty);
                float rs = s[nearest_index];
                float rx = tx[nearest_index];
                float ry = ty[nearest_index];
                float rtheta = tyaw[nearest_index];
                // 进行坐标转换
                std::pair<float, float> s_d = cartesian_to_frenet1D(rs, rx, ry, rtheta, obstacle_bb[u].first,
                                                                      obstacle_bb[u].second);
                ob_s[j][u] = s_d.first;
                ob_d[j][u] = s_d.second;
            }
        }
        std::pair<Mode, float> mode_stop_line_s = decision(s0, cars[0].s() - D0 - tau * cars[0].s_d(), ob_s, ob_d);
        Mode mode = mode_stop_line_s.first;
        stop_line_s = mode_stop_line_s.second;
        s1 = stop_line_s;

        if (mode == Mode::STOPPING) {
            auto stopping_start = std::chrono::high_resolution_clock::now();
            std::tie(path, all_paths) = frenet_stopping_planning(
                    csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s1, s1_d, s1_dd, ob);
            auto stopping_end = std::chrono::high_resolution_clock::now();
            auto stopping_duration = std::chrono::duration_cast<std::chrono::milliseconds>(stopping_end - stopping_start);

            std::cout << "Stopping Calculated Time: " << stopping_duration.count() << " ms" << std::endl;
        }
        else if (mode == Mode::FOLLOWING) {
            auto following_start = std::chrono::high_resolution_clock::now();
            std::tie(path, all_paths) = frenet_following_planning(
                    csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                    cars[0].s() - D0 - tau * cars[0].s_d(), cars[0].s_d(), cars[0].s_dd(), ob);
            auto following_end = std::chrono::high_resolution_clock::now();
            auto following_duration = std::chrono::duration_cast<std::chrono::milliseconds>(following_end - following_start);

            std::cout << "Following Calculated Time: " << following_duration.count() << " ms" << std::endl;
        }
        else {
            auto velocity_keeping_start = std::chrono::high_resolution_clock::now();
            std::tie(path, all_paths) = frenet_velocity_keeping_planning(
                    csp, s0, TARGET_SPEED, c_d, c_d_d, c_d_dd, ob);
            auto velocity_keeping_end = std::chrono::high_resolution_clock::now();
            auto velocity_keeping_duration = std::chrono::duration_cast<std::chrono::milliseconds>(velocity_keeping_end -
                    velocity_keeping_start);

            std::cout << "Velocity Keeping Calculated Time: " << velocity_keeping_duration.count() << " ms" << std::endl;
        }

        s0 = path.s[1];
        s0_d = path.s_d[1];
        s0_dd = path.s_dd[1];

        c_d = path.d[1];
        c_d_d = path.d_d[1];
        c_d_dd = path.d_dd[1];

        c_speed = path.s_d[0];

        for (auto &car: cars) {
            car.update(DT);
        }


        if (std::hypot(path.x[1] - tx.back(), path.y[1] - ty.back()) <= 1.0) {
            std::cout << "Goal" << std::endl;
            break;
        }

        if (std::abs(s0 - stop_line_s) < 0.2) {
            std::cout << "stop" << std::endl;
            break;
        }

        std::ofstream ofs("output_change_quintic.csv", std::ios::app);
//        std::ofstream ofs("output_change_quintic.csv", std::ios::trunc);
        ofs << "x, y, yaw, final_path.s, final_path.s_d, curvature" << std::endl;

        for (unsigned int w = 0; w < path.x.size(); w++) {
            ofs << std::setprecision(15) << path.x[w] << "," << std::setprecision(15) << path.y[w] << ","
                << std::setprecision(15) << path.yaw[w] << "," << std::setprecision(15) << path.s[w] << ","
                << std::setprecision(15) << path.s_d[w] << "," << std::setprecision(15) << path.c[w] << std::endl;
        }
        ofs.close();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Calculated Time: " << duration.count() << " ms" << std::endl;

    std::cout << "Finish" << std::endl;

    return 0;

}