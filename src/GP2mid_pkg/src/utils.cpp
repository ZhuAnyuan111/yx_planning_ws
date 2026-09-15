#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <GP2/utils.h>

struct Point{
    double x,y;
    Point(double x=0,double y=0):x(x),y(y){}

    //向量+
    Point operator +(const Point &b)const
    {
        return Point(x+b.x,y+b.y);
    }
    //向量-
    Point operator -(const Point &b)const
    {
        return Point(x-b.x,y-b.y);
    }
    //数乘*
    Point operator *(double k) const
    {
        return Point(x*k, y*k);
    }
    //点积
    double operator *(const Point &b)const
    {
        return x*b.x + y*b.y;
    }
    //叉积
    //P^Q>0,P在Q的顺时针方向；<0，P在Q的逆时针方向；=0，P，Q共线，可能同向或反向
    double operator ^(const Point &b)const
    {
        return x*b.y - b.x*y;
    }
    //长度平方
    double norm() const
    {
        return x*x + y*y;
    }
};

//三态函数，判断两个double在eps精度下的大小关系
int dcmp(double x)
{
    double eps=1e-6;
    if(fabs(x)<eps) return 0;
    else
        return x<0?-1:1;
}

//判断点Q是否在P1和P2的线段上
bool OnSegment(Point P1,Point P2,Point Q)
{
    return dcmp((P1-Q)^(P2-Q))==0&&dcmp((P1-Q)*(P2-Q))<=0;
}

//判断点P在多边形内-射线法
bool InPolygon(const Point& P,const std::vector<Point>& polygon)
{
    bool isInside = false;
    int n = polygon.size();
    Point P1,P2; //多边形一条边的两个顶点
    for(int i=0,j=n-1;i<n;j=i++)
    {
        P1 = polygon[i];
        P2 = polygon[j];
        if(OnSegment(P1,P2,P)) return true; //点在多边形一条边上
        if( (dcmp(P1.y-P.y)>0 != dcmp(P2.y-P.y)>0) && dcmp(P.x - (P.y-P1.y)*(P1.x-P2.x)/(P1.y-P2.y)-P1.x)<0)
            isInside = !isInside;
    }
    return isInside;
}

//向量P投影到直线AB
Point ProjectPointToLine(const Point &P, const Point &A, const Point &B) {
    Point AP = P - A;  // 从点 A 到点 P 的向量
    Point AB = B - A;  // 从点 A 到点 B 的向量
    double factor = (AP*AB) / AB.norm();  // 计算投影的长度比
    Point projection = A + AB * factor;  // 计算投影点
    return projection;
}

// 旋转网格
void rotate_grid(Eigen::MatrixXd& x_grid, Eigen::MatrixXd& y_grid, double angle)
{
    Eigen::MatrixXd x_temp = x_grid;
    Eigen::MatrixXd y_temp = y_grid;

    x_grid=cos(angle)*x_temp-sin(angle)*y_temp;
    y_grid=sin(angle)*x_temp+cos(angle)*y_temp;
}

// 寻找eigen array若干最小的元素和其索引
std::vector<std::pair<double, int>> findNmin(const Eigen::ArrayXd& arr, int n)
{
    // 创建一个存储元素和索引的向量
    std::vector<std::pair<double, int>> indexed_values;
    indexed_values.reserve(arr.size());

    // 填充向量
    for (int i = 0; i < arr.size(); ++i) {
        indexed_values.emplace_back(arr[i], i);
    }

    // 排序向量，基于元素值
    std::nth_element(indexed_values.begin(), indexed_values.begin() + n, indexed_values.end(),
                     [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                         return a.first < b.first;});
    
    return indexed_values;
}

Eigen::ArrayXd movmean(const Eigen::ArrayXd& data, int windowSize) // 向量滑动平均
{
    
    int n = data.size();
    Eigen::ArrayXd result=Eigen::ArrayXd::Zero(n);
    
     for (int i=0; i<n; i++)
    {
        double sum=0.0;
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