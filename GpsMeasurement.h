//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_GPSMEASUREMENT_H
#define PF_GPSMEASUREMENT_H


struct GpsMeasurement
{
    GpsMeasurement(float X, float Y, float Theta) : X(X), Y(Y), Theta(Theta) {}

    float X;
    float Y;
    // Heading
    float Theta;
};


#endif //PF_GPSMEASUREMENT_H
