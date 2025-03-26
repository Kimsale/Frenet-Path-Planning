/*************************************************************************
	> File Name: frenet_path.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Tue Apr  9 17:08:22 2019
 ************************************************************************/

#ifndef FRENET_PATH_H
#define FRENET_PATH_H

#include<iostream>
#include<vector>
#include<array>
#include<string>
#include"cpprobotics_types.h"

namespace cpprobotics{

class FrenetPath{
public:
  float cd = 0.0; // 横向（纵向）方向上的偏差代价
  float cv = 0.0; // 纵向（横向）方向上的速度代价
  float cf = 0.0; // 纵向（横向）方向上的舒适代价

  Vec_d t;
  Vec_d d;     // 参考线法线方向（横向）位移
  Vec_d d_d;   // 横向速度
  Vec_d d_dd;  // 横向加速度
  Vec_d d_ddd; // 横向加加速度
  Vec_d s;     // 沿参考线纵向位移
  Vec_d s_d;   // 沿参考线纵向速度
  Vec_d s_dd;  // 沿参考线纵向加速度
  Vec_d s_ddd; // 纵向加加速度

  Vec_d x; // 车辆在全局坐标系下的x坐标列表
  Vec_d y;
  Vec_d yaw;
  Vec_d ds;
  Vec_d c;// python is different
  Vec_d v;
  Vec_d a;

//  float max_speed;
//  float max_accel;
//  float max_curvature;
};

using Vec_Path=std::vector<FrenetPath>;

}
#endif
