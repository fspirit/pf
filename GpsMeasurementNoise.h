//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_GPSMEASUREMENTNOISE_H
#define PF_GPSMEASUREMENTNOISE_H


struct GpsMeasurementNoise
{
    GpsMeasurementNoise(float XVariance, float YVariance, float ThetaVariance) : XVariance(XVariance),
                                                                                 YVariance(YVariance),
                                                                                 ThetaVariance(ThetaVariance) {}

    float XVariance;
    float YVariance;
    float ThetaVariance;
};


#endif //PF_GPSMEASUREMENTNOISE_H
