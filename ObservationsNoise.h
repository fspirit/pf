//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_OBSERVATIONSNOISE_H
#define PF_OBSERVATIONSNOISE_H


struct ObservationsNoise
{
    ObservationsNoise(float XVar, float YVar) : XVar(XVar), YVar(YVar) {}

    float XVar;
    float YVar;
};


#endif //PF_OBSERVATIONSNOISE_H
