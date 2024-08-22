#pragma once
#include "main.h"
#include "odom.hpp"
#include "PID_Controller.hpp"
#include "util.hpp"


double absoluteAngleToPoint(const point& pos, const point& point)
{
    double t = 0;

    try
    { 
        t = atan2(point.x - pos.x, point.y - pos.y);
    }

    catch(...)
    {
        t = PI/2;
    }
    
    t = lib::rtd(t);

    // -270 - 90

    //-180 - 180

    t = -t;
    t = t >= 0 ? t :  180 + 180+t;
    return (t);
}

std::vector<double> pidMTPVel(const point& target, double rotationBias, lib::pid* lCont, lib::pid* rCont)
{
    double linearError = sqrt(pow(target.x - pos.x, 2) + pow(target.y - pos.y, 2));
    double currHeading = gyro.get_rotation();
    double targetHeading = absoluteAngleToPoint(pos, target);
    double rotationError = lib::minError(targetHeading,currHeading);
    double cre = abs(rotationError) > 90 ? 0.1 : cos(lib::dtr(rotationError));
    double angularVel = rCont -> out(rotationError);
    double linearVel = cre * lCont -> out(linearError);
    double rVel = (linearVel - (fabs(angularVel) * rotationBias)) + angularVel;
    double lVel = (linearVel - (fabs(angularVel) * rotationBias)) - angularVel;
    return(std::vector<double> {rVel, lVel});
}

void pidMoveTo(const point& target, double timeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias)
{
  lib::timer timeoutTimer;
  lib::pid linearController(lConstants, 0);
  lib::pid rotationController(rConstants, 0);

  while (timeoutTimer.time() < timeout) {
    std::vector<double> velocities = (pidMTPVel(target, rotationBias, &linearController, &rotationController));
    driveRightGroup.moveVoltage(velocities[0]);
    driveLeftGroup.moveVoltage(velocities[1]);
  }
  driveRightGroup.moveVoltage(0);
  driveLeftGroup.moveVoltage(0);
}



point targetPoint(std::vector<point> path, int lookAhead, int lineLookAhead, int lineIndex){

    point targetPoint;
    double closestDist = 1000000000;
    
    int a = lineLookAhead < (path.size() - lineIndex) ? lineIndex+lineLookAhead : path.size() -1;
    for (int i = lineIndex; i <= a; i++)
    {    
        // curr points
        point p1 = path.at(i);
        point p2 = path.at(i+1);

        // offset to origin
        p1.x -= pos.x;
        p1.y -= pos.y;
        p2.x -= pos.x;
        p2.y -= pos.y;
        // set up to find intersection using this method 
        // https://mathworld.wolfram.com/Circle-LineIntersection.html
        double dx = p2.x-p1.x;
        double dy = p2.y-p1.y;
        double dr = sqrt(pow(dx,2)+pow(dy,2));
        double D = p1.x*p2.y - p2.x * p1.y;

        double discriminant = pow(lookAhead,2)  *  pow(dr,2) - pow(D,2);

        // calculate solutions
        if (discriminant >= 0)
        {
            double sDiscriminant = sqrt(discriminant);
            double dxdy = D * dy;
            double dxdx = D*dx;
            double sdyxdxxsd = squiggles::sgn(dy) * dx * sDiscriminant;
            double dr2 = pow(dr,2);
            double adyxsd = std::abs(dy) * sDiscriminant;
            point farthestPoint = path.at(a-1);

            int minX = std::min(p1.x,p2.x);
            int maxX = std::max(p1.x,p2.x);
            int minY = std::min(p1.y,p2.y);
            int maxY = std::max(p1.y,p2.y);

            double sx1 = (dxdy + sdyxdxxsd) / dr2;
            double sy1 = (-dxdx + adyxsd) / dr2;
            double sx2 = (dxdy - sdyxdxxsd) / dr2;
            double sy2 = (-dxdx - adyxsd) / dr2;

            point s1 = {sx1 + pos.x, sy1 + pos.y};
            point s2 = {sx2 + pos.x, sy2 + pos.y};
            
            bool s1Valid = s1.x >= minX && s1.x <= maxX && s1.y >= minY && s1.y <= maxY;
            bool s2Valid = s2.x >= minX && s2.x <= maxX && s2.y >= minY && s2.y <= maxY;

            double s1Dist = dist(s1, farthestPoint);
            double s2Dist = dist(s2,farthestPoint);

            if (s1Valid && s1Dist < closestDist)
            {
                targetPoint = s1;
                closestDist = s1Dist;
            }

            if (s2Valid && s2Dist < closestDist)
            {
                targetPoint = s2;
                closestDist = s2Dist;
            }

        }

    } 

    return(targetPoint);
}

double dist (point &p1, point &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// distToPoint(x, y, point.getX(), point.getY()) < radius

void moveToPurePursuit(std::vector<point> path, double lookAhead, int lineLookAhead, int finalTimeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias)
{

    bool targetReached = false;
    int lineIndex = 0;
    point target;
    lib::pid linearController(lConstants, 0);
    lib::pid rotationController(rConstants, 0);

    while (!targetReached)
    {

        if ((sqrt(pow((path[lineIndex + lineLookAhead]).y - pos.y, 2) + pow((path[lineIndex + lineLookAhead]).x - pos.x, 2))) < lookAhead)
        {
            lineIndex += 1;
        }

        if ((sqrt(pow((path[path.size()-1]).y - pos.y, 2) + pow((path[path.size()-1]).x - pos.x, 2))) < lookAhead)
        {
            targetReached = true;
        }

        target = targetPoint(path,lookAhead, lineLookAhead, lineIndex);
        
        std::vector<double> velocities = (pidMTPVel(target, rotationBias, &linearController, &rotationController));

    }

   pidMoveTo(target, finalTimeout, lConstants, rConstants, rotationBias);
}   
