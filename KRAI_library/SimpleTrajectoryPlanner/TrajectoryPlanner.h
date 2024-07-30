#pragma once
#include <cmath>

class TrajectoryPlanner {
private:
    bool doForceBrake;
    bool doTrajectory;
    float prevPose;
    float xSign;

public:
    TrajectoryPlanner();
    float Phi(float x);
    float Theta(float x);
    float Lx(float x, float aVal, float bVal);
    float interpolation(float x, float lengthX, float v0, float vMax, bool allowForceBrake, float aRange, float bRangeStart, float bRangeFinal, float errorTol, float errorFinalTol);
    bool checkForceBrake();

    void setStatusTraj(bool x) { this->doTrajectory = x; }
    void setPrevPose(float x) { this->prevPose = x; }
    void setBrakeStatus(bool x) { this->doForceBrake = x; }

    bool getStatusTraj() { return this->doTrajectory; }
    float getPrevPose() { return this->prevPose; }
    bool getBrakeStatus() { return this->doForceBrake; }
};