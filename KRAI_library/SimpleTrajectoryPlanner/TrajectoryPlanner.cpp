#include "TrajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner()
{
    this->doForceBrake = false;
    this->doTrajectory = false;
    this->prevPose = 0;
    this->xSign = 1;
}

float TrajectoryPlanner::Phi(float x)
{
    if (x > 0) {
        return exp(-1/x);
    }
    return 0;
}

float TrajectoryPlanner::Theta(float x)
{
    if (x <= 0) {
        return 0;
    }
    else if (x < 1) {
        return Phi(x) / (Phi(x) + Phi(1-x));
    }
    else if (x >= 1) {
        return 1;
    }
    return 0;
}

float TrajectoryPlanner::Lx(float x, float aVal, float bVal)
{
    return (x - aVal) / (bVal - aVal);
}

float TrajectoryPlanner::interpolation(float x, float lengthX, float v0, float vMax, bool allowForceBrake, float aRange, float bRangeStart, float bRangeFinal, float errorTol, float errorFinalTol)
{
    if (allowForceBrake) {
        if (x > lengthX && x <= (lengthX + errorTol)) {
            this->doForceBrake = true;
            return 0;
        }

        else {
            this->doForceBrake = false;
        }
    }

    if (lengthX > 0) {
        if (x < 0) {
            return v0;
        }
    }
    else if (lengthX < 0) {
        if (x > 0) {
            return -v0;
        }
    }

    if (x > 0) {
        this->xSign = 1;
    }
    else if (x < 0) {
        this->xSign = -1;
    }
    x = fabs(x);
    lengthX = fabs(lengthX);

    float prevLength = lengthX;
    lengthX = lengthX - ((bRangeStart + bRangeFinal)/2.0f - errorTol - aRange);
    return this->xSign * (((1 - Theta(Lx(x, aRange, bRangeStart))) * v0 + Theta(Lx(x, aRange, bRangeStart)) * vMax) - (Theta(Lx(x - lengthX, aRange, bRangeFinal)) * vMax) - ( Theta(Lx(x - prevLength - errorFinalTol, aRange, bRangeStart)) * vMax));
}

bool TrajectoryPlanner::checkForceBrake()
{
    return this->doForceBrake;
}