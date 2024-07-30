#pragma once

class MovingAverage
{
private:
    float *placeHolderArr;
    int stepSize = 0;
    int indexCounter = 0;
    bool startCalculate = false;

public:
    MovingAverage(int movAvgStep);
    ~MovingAverage();
    float calculateAvg(float *placeHolder);
    float movingAverage(float data);
};