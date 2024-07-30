#include "MovingAverage.h"

MovingAverage::MovingAverage(int movAvgStep)
{
    this->stepSize = movAvgStep;
    this->placeHolderArr = new float[movAvgStep];
}

MovingAverage::~MovingAverage()
{
    delete[]placeHolderArr;
    placeHolderArr = nullptr;
}

float MovingAverage::calculateAvg(float *placeHolder)
{
    float sum = 0.0f;
    for (int i = 0; i < stepSize; i++)
    {
        sum += placeHolder[i];
    }
    return sum/(float)this->stepSize;
}

float MovingAverage::movingAverage(float data)
{
    float firstSum = 0.0f;

    if (!startCalculate)
    {
        this->placeHolderArr[indexCounter] = data;
        indexCounter++;

        if (indexCounter == stepSize)
        {
            startCalculate = true;
            indexCounter = 0;
            return calculateAvg(this->placeHolderArr);
        }

        for (int i = 0; i < stepSize; i++)
        {
            firstSum += this->placeHolderArr[i];
        }

        return firstSum/(float)indexCounter;
    }
    else
    {
        this->placeHolderArr[indexCounter] = data;
        indexCounter++;

        if (indexCounter == stepSize)
        {
            startCalculate = true;
            indexCounter = 0;
        }
        return calculateAvg(this->placeHolderArr);
    }
}