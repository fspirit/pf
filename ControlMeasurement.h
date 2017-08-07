//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_CONTROLMEASUREMENT_H
#define PF_CONTROLMEASUREMENT_H


struct ControlMeasurement
{
    ControlMeasurement(float Velocity, float YawRate) : Velocity(Velocity), YawRate(YawRate) {}

    float Velocity;
    float YawRate;
};


#endif //PF_CONTROLMEASUREMENT_H
