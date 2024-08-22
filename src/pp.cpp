#include "util.h"
#include "main.h"
#include "odom.hpp"
#include <cmath>
#include <vector>
#include <algorithm>



lib::cubicBezier::cubicBezier(const point& p0, const point& p1, const point& p2, const point& p3) 
    : p0(p0), p1(p1), p2(p2), p3(p3) {}

point lib::cubicBezier::evaluate(double t) 
{
    double omt = 1 - t;
    double omt2 = omt * omt;
    double omt3 = omt2 * omt;
    double t2 = t * t;
    double t3 = t2 * t;
    double x = omt3 * p0.x + 3 * omt2 * t * p1.x + 3 * omt * t2 * p2.x + t3 * p3.x;
    double y = omt3 * p0.y + 3 * omt2 * t * p1.y + 3 * omt * t2 * p2.y + t3 * p3.y;
    return {x, y};
}

point lib::cubicBezier::evaluateDerivative(double t) 
{
    double omt = 1 - t;
    double omt2 = omt * omt;
    double t2 = t * t;
    double x = 3 * omt2 * (p1.x - p0.x) + 6 * omt * t * (p2.x - p1.x) + 3 * t2 * (p3.x - p2.x);
    double y = 3 * omt2 * (p1.y - p0.y) + 6 * omt * t * (p2.y - p1.y) + 3 * t2 * (p3.y - p2.y);
    return {x, y, 0};
}

double lib::cubicBezier::length(int reso) 
{
    double result = 0;
    point prev = p0;
    for (int i = 1; i <= reso; ++i) {
        double t = static_cast<double>(i) / reso;
        point curr = evaluate(t);
        result += std::sqrt(std::pow(curr.x - prev.x, 2) + std::pow(curr.y - prev.y, 2));
        prev = curr;
    }
    return result;
}

double lib::cubicBezier::maxDeriv()
{
    return std::max(evaluateDerivative(0.00000001).x, evaluateDerivative(1).x);
}

lib::linearPath::linearPath(std::vector<point> points) 
    : points(points), length(points.size()) {}

int lib::linearPath::len()
{
    return length;
}

point lib::linearPath::at(int index)
{
    return points[index];
}

point lib::linearPath::last()
{
    return points[length - 1];
}
