//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_LANDMARKOBSERVATION_H
#define PF_LANDMARKOBSERVATION_H


struct LandmarkObservation
{
    LandmarkObservation(float X, float Y) : X(X), Y(Y) {}
    LandmarkObservation(int Id, float X, float Y) : Id(Id), X(X), Y(Y) {}

    int Id;
    float X;
    float Y;
};


#endif //PF_LANDMARKOBSERVATION_H
