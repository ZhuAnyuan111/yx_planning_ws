#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

template <typename T>
bool contains(const std::vector<T>& vec, T value)            // 向量是否包含某元素
{
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}

template <typename T>
bool containsNaN(const std::vector<T>& vec)         // 是否包含NAN值
{
    for (T num: vec)
    {
        if (isnan(num))
            return true;
    }
    return false;
}

template <typename T>
bool isZeroVector(const std::vector<T>& vec) 
{
    for (const T& value : vec) {
        if (value != 0) {
            return false; // 如果找到任何非零元素，返回 false
        }
    }
    return true; // 所有元素都是零，返回 true
}

template <typename T>
std::vector<T> movmean(const std::vector<T>& data, int windowSize)       // 计算滑动平均
{
    std::vector<T> result;
    int n = data.size();
    result.assign(n,0);

    for (int i=0; i<n; i++)
    {
        T sum=0.0;
        int count=0;
        for (int j = i - std::floor(windowSize/2); j < i + std::floor(windowSize+1/2); j++)
        {
            if (j>=0 && j<n)
            {
                sum = sum + data[j];
                count++;
            }
        }
        result[i] = sum/count;    
    }
    return result;
}
Eigen::ArrayXd movmean(const Eigen::ArrayXd& data, int windowSize);

template <typename T>
double rad2deg(T x)             // rad -> deg
{
    return static_cast<double>(x)/M_PI*180;
}

template <typename T>
double deg2rad(T x)             // deg -> rad
{
    return static_cast<double>(x)/180*M_PI;
}

inline double wrapTo2Pi(double a)
{
    a = std::fmod(a, 2 * M_PI);
    if (a < 0) a += 2 * M_PI;
    return a;
}

inline double angleDiffSigned(double to, double from)
{
    // return signed shortest difference (to - from) in [-pi, pi]
    double d = wrapTo2Pi(to) - wrapTo2Pi(from);
    if (d > M_PI) d -= 2 * M_PI;
    if (d < -M_PI) d += 2 * M_PI;
    return d;
}

template <typename T>
void printvec(const std::vector<T>& vec)    // 打印std vector
{
    for (int i=0; i< vec.size(); i++)
    {
        std::cout << vec[i] << " ";
    }
    std::cout << std::endl;
}

template<typename T>
void printeig(const T& matrix) {
    // 循环遍历矩阵或向量的每一个元素
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            std::cout << matrix(i, j);  // 访问并打印元素
            if (j != matrix.cols() - 1) std::cout << ", ";  // 元素之间加逗号分隔
        }
        std::cout << std::endl;  // 每行结束后换行
    }
}

template<typename T>
std::vector<T> ExpInterp(T x1, T x2, T y1, T y2,int num, double expnt)         // 指数插值，x1<x2, 在[x1,x2]内均匀插值:x1,x1+d,x1+2d,...,x2共num个数
{   
    std::vector<T> out;
    out.assign(num,0);
    if (num<2 || (x1 == x2 && expnt==0) )
    {
        std::cout << "ExpInterp ERROR \n" << std::endl;
        return out;
    }

    // 插值系数
    T coef = (y1-y2)/pow(x1-x2,expnt);
    // 插值
    for (int i=0; i<num; i++)
    {
        //这里是镜像指数插值。y = y2 + (y1 - y2) * pow((x - x2)/(x1 - x2),expnt);
        out[i] = y2 + coef * pow((static_cast<double>(i) - num + 1)/(num - 1) * (x2 - x1),expnt);
    }
    return out;
}

// T 是一般类型的变量
template<typename T>
// LinInterp1 是一维线性插值函数
T LinInterp1(const Eigen::Matrix<T , -1 , 1>& x,const Eigen::Matrix<T , -1 , 1>& y,
                        const T& xq)                // 一维线性插值， x已排序
 {
    T out=std::numeric_limits<T>::quiet_NaN();
    // 检查输入参数的有效性
    if (x.size() != y.size()  || x.size()==0)
    {
        std::cout << "LinInterp1 error! \n"; 
        return out; 
    }
        
    // 遍历所有可能的 i 值，查找 xq 在哪个区间内
    for (int i=0; i<x.size()-1; i++)
    {
        // 如果在区间内，进行插值
        if (x[i] <= xq && xq <= x[i+1] && x[i] != x[i+1])
        {
            out = (xq - x[i]) / (x[i+1] - x[i]) * y[i] + (x[i+1] - xq) / (x[i+1] - x[i]) * y[i+1];
            break;
        }
    }
    return out;
 }

template<typename T>
std::pair<std::vector<T>,std::vector<T>> LinSegment(T x1, T x2, T y1, T y2, int num)      // 线段内插值: x1, x1 + d, x1 + 2d, ... , x2 共num个点一线，返回对应y值向量
{
    std::pair<std::vector<T>,std::vector<T>> out;
    std::vector<T> outX;
    std::vector<T> outY;
    if (num <=1 )
    {
        // num <=1 时不采样
        std::cout << "LinSegment ERROR \n";
        outX.push_back(std::numeric_limits<T>::quiet_NaN());
        outY.push_back(std::numeric_limits<T>::quiet_NaN());
        out.first = outX; out.second = outY;
        return out;
    }
    
    for (int i = 0; i < num; i++)
    {
        // 计算第i个点的x值
        T x_i = (static_cast<T>( num - 1 - i))/(static_cast<T>(num - 1)) * x1 + (static_cast<T>(i))/(static_cast<T>(num - 1)) * x2;
        outX.push_back(x_i);
        // 计算第i个点的y值
        T y_i = (static_cast<T>( num - 1 - i))/(static_cast<T>(num - 1)) * y1 + (static_cast<T>(i))/(static_cast<T>(num - 1)) * y2;
        outY.push_back(y_i);
    }
    out.first = outX; out.second = outY;
    return out;
}

template<typename T>
T Saturate(T x, T ub, T lb)         // 把一个数限制在[lb,ub]内
{
    ub = std::max(ub,lb);
    lb = std::min(ub,lb);

    T y = ( x >= ub) * ub + ( x <= lb) * lb + ( x < ub && x > lb) * x;
    return y;
}

struct Point;
int dcmp(double x);
bool OnSegment(Point P1,Point P2,Point Q);
bool InPolygon(const Point& P,const std::vector<Point>& polygon);
Point ProjectPointToLine(const Point &P, const Point &A, const Point &B);
void rotate_grid(Eigen::MatrixXd& x_grid, Eigen::MatrixXd& y_grid, double angle);
std::vector<std::pair<double, int>> findNmin(const Eigen::ArrayXd& arr, int n);

#endif