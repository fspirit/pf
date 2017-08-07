//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_LANDMARK_H
#define PF_LANDMARK_H

#include <vector>

struct Landmark
{
    Landmark(int Id, float X, float Y) : Id(Id), X(X), Y(Y) {}

    static std::vector<Landmark> ReadLandmarksFromFile();

    int GetId() const { return Id; }
    float GetX() const { return X; }
    float GetY() const { return Y; }

private:
    int Id;
    float X;
    float Y;
};


#endif //PF_LANDMARK_H
