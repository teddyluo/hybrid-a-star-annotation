/**  FILE: dubins.cpp
 * @brief Dubins路径生成类的实现，用于生成向前行走的轨迹
 * 注：Dubins路径是基础中的基础，扩展的Reeds-Shepp路径是Dubins路径的扩展（包括后退方向）
 *    Dubins路径遵循车辆运动学约束。这里使用的代码直接采用Andrew Walker编写的Dubins类：
 *  https://github.com/AndrewWalker/Dubins-Curves/
 */

// Copyright (c) 2008-2014, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DUBINS_CPP
#define DUBINS_CPP

#include "dubins.h"
#define _USE_MATH_DEFINES // for C++
#include <math.h>
#include <assert.h>

#define EPSILON (10e-10)

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

namespace HybridAStar{

// The segment types for each of the Path types
const int DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

//函数指针数组，可以指向不同的函数
//原型为 typedef int (*DubinsWord)(double, double, double, double* );
DubinsWord dubins_words[] = {
    dubins_LSL,
    dubins_LSR,
    dubins_RSL,
    dubins_RSR,
    dubins_RLR,
    dubins_LRL,
};

//这里定义宏展开，方便简化代码
#define UNPACK_INPUTS(alpha, beta)     \
    double sa = sin(alpha);            \
    double sb = sin(beta);             \
    double ca = cos(alpha);            \
    double cb = cos(beta);             \
    double c_ab = cos(alpha - beta);   \

//定义宏展开，方便简化代码
#define PACK_OUTPUTS(outputs)       \
    outputs[0]  = t;                \
    outputs[1]  = p;                \
    outputs[2]  = q;

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
/**
 * 两浮点数取模
 */
double fmodr( double x, double y)
{
    return x - y*floor(x/y);
}

/**
 * @brief 将角度范围限制在（-2pi, 2*pi）之间
 * 
 * @param theta 角度
 * @return double 计算结果
 */
double mod2pi( double theta )
{
    return fmodr( theta, 2 * M_PI );
}

/**
 * @brief 根据alpha, beta, d三个参数找出Dubins类型(LSL, LSR, RSL, RSR, RLR or LRL)
 *        并将每段曲线的线段长度（代价）存放到path->param和path->type中
 *       注：这是一种简单的穷举尝试策略，与文章“Classification of the Dubins set”描述的策略不同。
 * 
 * @param alpha ：角度参数，表示起点朝向
 * @param beta ：角度参数，表示终点朝向
 * @param d ：起点和终点的距离
 * @param path 存放结果，用到两个: path->param和path->type
 * @return int 返回值：非0值表示出错；0表示正常
 */
int dubins_init_normalised( double alpha, double beta, double d, DubinsPath* path)
{
    double best_cost = INFINITY;
    int    best_word;
    int    i;

    best_word = -1;
    for( i = 0; i < 6; i++ ) {
        double params[3];
        //分别调用不同的Dubins函数曲线计算得到t,p, q,存放于params
        int err = dubins_words[i](alpha, beta, d, params);
        if(err == EDUBOK) {
            double cost = params[0] + params[1] + params[2];//三段线段的长度作为代价
            if(cost < best_cost) {//将最好的结果保存到path->param看
                best_word = i;
                best_cost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = i;
            }
        }
    }

    if(best_word == -1) {
        return EDUBNOPATH;
    }
    path->type = best_word;
    return EDUBOK;
}

/**
 * @brief 依据始点、末点位置和最小转弯半径生成一条Dubin PATH
 * 
 * @param q0 始点
 * @param q1 末点
 * @param rho 最小转弯半径
 * @param path 生成的路径
 * @return int 
 */
int dubins_init( double q0[3], double q1[3], double rho, DubinsPath* path )
{
    int i;
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double D = sqrt( dx * dx + dy * dy );
    double d = D / rho;//将两点距离除以最小转弯半径得到归一化距离d
    if( rho <= 0. ) {
        return EDUBBADRHO;
    }
    double theta = mod2pi(atan2( dy, dx ));
    double alpha = mod2pi(q0[2] - theta);
    double beta  = mod2pi(q1[2] - theta);//计算坐标原点移到第一个点后，始点和终点的朝向
    for( i = 0; i < 3; i ++ ) {//将起点放入path->qi变量
        path->qi[i] = q0[i];
    }
    path->rho = rho;//最小转弯半径

    return dubins_init_normalised( alpha, beta, d, path );
}

int dubins_LSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double tmp0 = d+sa-sb;
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    double tmp1 = atan2( (cb-ca), tmp0 );
    double t = mod2pi(-alpha + tmp1 );
    double p = sqrt( p_squared );
    double q = mod2pi(beta - tmp1 );
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double tmp0 = d-sa+sb;
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    double tmp1 = atan2( (ca-cb), tmp0 );
    double t = mod2pi( alpha - tmp1 );
    double p = sqrt( p_squared );
    double q = mod2pi( -beta + tmp1 );
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_LSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double p_squared = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    double p    = sqrt( p_squared );
    double tmp2 = atan2( (-ca-cb), (d+sa+sb) ) - atan2(-2.0, p);
    double t    = mod2pi(-alpha + tmp2);
    double q    = mod2pi( -mod2pi(beta) + tmp2 );
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double p_squared = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
    if( p_squared< 0 ) {
        return EDUBNOPATH;
    }
    double p    = sqrt( p_squared );
    double tmp2 = atan2( (ca+cb), (d-sa-sb) ) - atan2(2.0, p);
    double t    = mod2pi(alpha - tmp2);
    double q    = mod2pi(beta - tmp2);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RLR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
    if( fabs(tmp_rlr) > 1) {
        return EDUBNOPATH;
    }
    double p = mod2pi( 2*M_PI - acos( tmp_rlr ) );
    double t = mod2pi(alpha - atan2( ca-cb, d-sa+sb ) + mod2pi(p/2.));
    double q = mod2pi(alpha - beta - t + mod2pi(p));
    PACK_OUTPUTS( outputs );
    return EDUBOK;
}

int dubins_LRL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;
    if( fabs(tmp_lrl) > 1) {
        return EDUBNOPATH;
    }
    double p = mod2pi( 2*M_PI - acos( tmp_lrl ) );
    double t = mod2pi(-alpha - atan2( ca-cb, d+sa-sb ) + p/2.);
    double q = mod2pi(mod2pi(beta) - alpha -t + mod2pi(p));
    PACK_OUTPUTS( outputs );
    return EDUBOK;
}

/**
 * @brief 计算路径长度
 * 
 * @param path ：三个参数表示角度
 * @return double ：返回值，表示路径长度
 */
double dubins_path_length( DubinsPath* path )
{
    double length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    length = length * path->rho;
    return length;
}

/**
 * @brief 返回路径类型
 * 
 * @param path 输入：路径
 * @return int 返回值：路径类型
 */
int dubins_path_type( DubinsPath* path ) {
    return path->type;
}

/**
 * @brief 计算下一段Dubins路径段的位置
 * 
 * @param t 角度 
 * @param qi 该段起始位置
 * @param qt 该段终点位置
 * @param type 路径类型
 */
void dubins_segment( double t, double qi[3], double qt[3], int type)
{
    assert( type == L_SEG || type == S_SEG || type == R_SEG );
    //Shkel A M, Lumelsky V. Classification of the Dubins set[J]. Robotics and Autonomous Systems, 2001, 34(4): 179-202.
    if( type == L_SEG ) {//公式 (1)的第一个式子，此处经过归一化后v=1
        qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2]+t) + cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if( type == R_SEG ) {//公式(1)的第二个式子，此处经过归一化后v=1
        qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2]-t) - cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else if( type == S_SEG ) {//公式(1)的第三个式子，此处经过归一化后v=1
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
    }
}

/**
 * @brief 对一段路径进行采样(根据路径长度t和路径参数path,生成长度为t的节点的位置)
 * 
 * @param path 输入路径参数
 * @param t 路径长度t
 * @param q 采样结果(线段终点的位置)
 * @return int 返回值：非0表示出错，0表示正常
 */
int dubins_path_sample( DubinsPath* path, double t, double q[3] )
{
    if( t < 0 || t >= dubins_path_length(path) ) {
        // error, parameter out of bounds
        return EDUBPARAM;
    }

    // tprime is the normalised variant of the parameter t
    double tprime = t / path->rho;

    // In order to take rho != 1 into account this function needs to be more complex
    // than it would be otherwise. The transformation is done in five stages.
    //
    // 1. translate the components of the initial configuration to the origin
    // 2. generate the target configuration
    // 3. transform the target configuration
    //      scale the target configuration
    //      translate the target configration back to the original starting point
    //      normalise the target configurations angular component

    // The translated initial configuration
    // 将路径放在原点(0, 0)，此时，只需要将角度保留即可
    double qi[3] = { 0, 0, path->qi[2] };

    // Generate the target configuration
    // 生成中间点的位置
    const int* types = DIRDATA[path->type];
    double p1 = path->param[0];//路径的第一个角度
    double p2 = path->param[1];//路径的第二个角度
    double q1[3]; // end-of segment 1
    double q2[3]; // end-of segment 2
    //从第qi点为起点，根据类型types[0]，生成后一个点q1的configuration（即计算下一个节点的位置）
    dubins_segment( p1,      qi,    q1, types[0] );
    //从第q1点为起点，根据类型types[1]，生成后一个点q2的configuration（即计算下一个节点的位置）
    dubins_segment( p2,      q1,    q2, types[1] );
    //生成q点的configuration
    if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );
    }
    else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
    }

    // scale the target configuration, translate back to the original starting point
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    q[2] = mod2pi( q[2] );

    return 0;
}

/**
 * @brief 通过回调函数多次调用dubins_path_sample
 * 
 * @param path 
 * @param cb 
 * @param stepSize 
 * @param user_data 
 * @return int 
 */
int dubins_path_sample_many( DubinsPath* path, DubinsPathSamplingCallback cb, double stepSize, void* user_data )
{
    double x = 0.0;
    double length = dubins_path_length(path);
    while( x <  length ) {
        double q[3];
        dubins_path_sample( path, x, q );
        int retcode = cb(q, x, user_data);
        if( retcode != 0 ) {
            return retcode;
        }
        x += stepSize;
    }
    return 0;
}

/**
 * @brief 根据路径参数path, 返回线段末节点参数
 * 
 * @param path 
 * @param q 
 * @return int 
 */
int dubins_path_endpoint( DubinsPath* path, double q[3] )
{
    // TODO - introduce a new constant rather than just using EPSILON
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON, q );
}

/**
 * @brief 将一段路径path，按路径长度t，划分为多个子线段。
 * 
 * @param path 输入的路径
 * @param t 子线段长度
 * @param newpath 生成的新的路径
 * @return int 
 */
int dubins_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath )
{
    // calculate the true parameter
    double tprime = t / path->rho;

    // copy most of the data
    newpath->qi[0] = path->qi[0];
    newpath->qi[1] = path->qi[1];
    newpath->qi[2] = path->qi[2];
    newpath->rho   = path->rho;
    newpath->type  = path->type;

    // fix the parameters
    newpath->param[0] = fmin( path->param[0], tprime );
    newpath->param[1] = fmin( path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = fmin( path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}
}
#endif
