/*************************************************************************
	> File Name: cpprobotics_types.h
	> Author: TAI Lei
	> Mail: ltai@ust.hk
	> Created Time: Mon Apr 22 13:25:12 2019
 ************************************************************************/

#ifndef CPPROBOTICS_TYPES_H
#define CPPROBOTICS_TYPES_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace cpprobotics{

using Vec_d=std::vector<float>;
using Poi_d=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_d>;

}

#endif
