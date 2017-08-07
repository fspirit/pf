//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_PARTICLEFILTER_H
#define PF_PARTICLEFILTER_H

#include <utility>
#include <vector>
#include <random>

#include "GpsMeasurement.h"
#include "ControlMeasurement.h"
#include "LandmarkObservation.h"
#include "GpsMeasurementNoise.h"
#include "ObservationsNoise.h"
#include "Landmark.h"

struct Particle
{
    Particle(float X, float Y, float Theta) : X(X), Y(Y), Theta(Theta), Weight(1.0) {}

    float X;
    float Y;
    float Theta;
    float Weight;
    std::vector<LandmarkObservation> LandmarkAssosiations;
};

class ParticleFilter
{
public:
    ParticleFilter(const GpsMeasurementNoise &gpsMeasurementNoise,
                   const ObservationsNoise &landmarkObservationNoise,
                   std::vector<Landmark> landmarks,
                   int numberOfParticles) :
            gpsMeasurementNoise(gpsMeasurementNoise),
            landmarkObservationNoise(landmarkObservationNoise),
            landmarks(std::move(landmarks)),
            isInitialised(false),
            numberOfParticles(numberOfParticles) {}

    void Update(GpsMeasurement gps,  ControlMeasurement u, std::vector<LandmarkObservation> z);
    Particle GetBestParticle();

    // Test

    void TestPointConversion();
    void TestPointsAssociation();
    void TestWeightsCalculation();

private:
    GpsMeasurementNoise gpsMeasurementNoise;
    ObservationsNoise landmarkObservationNoise;
    std::vector<Landmark> landmarks;
    bool isInitialised;
    int numberOfParticles;
    std::vector<Particle> particles;
    std::default_random_engine randomEngine;

    void InitialiseWithGpsMeasurement(GpsMeasurement measurement);
    void Predict(ControlMeasurement u);
    void Resample();
    void TransformCoordinates(Particle &p,
                              std::vector<LandmarkObservation> &observations) const;
    void MapObservationsToLandmarks(std::vector<LandmarkObservation> &observations) const;
    float CalculateObservationsProbability(std::vector<LandmarkObservation> &observations);
    void UpdateWeights(std::vector<LandmarkObservation> &z);
    static double GetDistance(LandmarkObservation const& observation, Landmark const& landmark);
};


#endif //PF_PARTICLEFILTER_H
