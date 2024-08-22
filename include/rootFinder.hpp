#ifndef ROOTFINDER_HPP
#define ROOTFINDER_HPP
#include <cfloat>
#include <cmath>
#include <set>

/**
 * @brief cubic equation solver
 * 
 * @param a cubic coefficient
 * @param b quadratic coefficient
 * @param c linear coefficient
 * @param d constant
 * @return std::set<double> set of all the roots
 */
std::set<double> solveCubicEquation(double a, double b, double c, double d)
{
    std::set<double> roots;

    constexpr double cos120 = -0.50;
    constexpr double sin120 = 0.866025403784438646764;

    if (fabs(d) < DBL_EPSILON)
    {
        // First solution is x = 0
        roots.insert(0.0);

        // Converting to a quadratic equation
        d = c;
        c = b;
        b = a;
        a = 0.0;
    }

    if (fabs(a) < DBL_EPSILON)
    {
        if (fabs(b) < DBL_EPSILON)
        {
            // Linear equation
            if (fabs(c) > DBL_EPSILON)
                roots.insert(-d / c);
        }
        else
        {
            // Quadratic equation
            double discriminant = c * c - 4.0 * b * d;
            if (discriminant >= 0)
            {
                double inv2b = 1.0 / (2.0 * b);
                double sqrtDiscriminant = sqrt(discriminant);
                roots.insert((-c + sqrtDiscriminant) * inv2b);
                roots.insert((-c - sqrtDiscriminant) * inv2b);
            }
        }
    }
    else
    {
        // Cubic equation
        double invA = 1.0 / a;
        double invASquared = invA * invA;
        double bSquared = b * b;
        double bOver3A = b * (1.0 / 3.0) * invA;
        double p = (3.0 * a * c - bSquared) * (1.0 / 3.0) * invASquared;
        double halfQ = (2.0 * bSquared * b - 9.0 * a * b * c + 27.0 * a * a * d) * (0.5 / 27.0) * invASquared * invA;
        double discriminant = p * p * p / 27.0 + halfQ * halfQ;

        if (discriminant > DBL_EPSILON)
        {
            // Sqrt is positive: one real solution
            double sqrtDiscriminant = sqrt(discriminant);
            double u = -halfQ + sqrtDiscriminant;
            double v = -halfQ - sqrtDiscriminant;
            double w = fabs(u) > fabs(v) ? u : v;
            double cubeRootW = (w < 0) ? -pow(fabs(w), 1.0 / 3.0) : pow(w, 1.0 / 3.0);
            roots.insert(cubeRootW - p / (3.0 * cubeRootW) - bOver3A);
        }
        else if (discriminant < -DBL_EPSILON)
        {
            // Sqrt is negative: three real solutions
            double realPart = -halfQ;
            double imaginaryPart = sqrt(-discriminant);
            double theta;
            double radius;
            double realComponent;
            double imaginaryComponent;
            // Convert to polar form
            if (fabs(realPart) > DBL_EPSILON)
            {
                theta = (realPart > 0.0) ? atan(imaginaryPart / realPart) : (atan(imaginaryPart / realPart) + 3.14159);
                radius = sqrt(realPart * realPart - discriminant);
            }
            else
            {
                // Vertical line
                theta = 3.14159 / 2.0;
                radius = imaginaryPart;
            }
            // Calculate cube root
            theta /= 3.0;
            radius = pow(radius, 1.0 / 3.0);
            // Convert to complex coordinate
            realComponent = cos(theta) * radius;
            imaginaryComponent = sin(theta) * radius;
            // First solution
            roots.insert(realComponent + realComponent - bOver3A);
            // Second solution, rotate +120 degrees
            roots.insert(2.0 * (realComponent * cos120 - imaginaryComponent * sin120) - bOver3A);
            // Third solution, rotate -120 degrees
            roots.insert(2.0 * (realComponent * cos120 + imaginaryComponent * sin120) - bOver3A);
        }
        else
        {
            // Sqrt is zero: two real solutions
            double w = -halfQ;
            double cubeRootW = (w < 0.0) ? -pow(fabs(w), 1.0 / 3.0) : pow(w, 1.0 / 3.0);
            // First solution
            roots.insert(cubeRootW + cubeRootW - bOver3A);
            // Second solution, rotate +120 degrees
            roots.insert(2.0 * cubeRootW * cos120 - bOver3A);
        }
    }
    return roots;
}
#endif