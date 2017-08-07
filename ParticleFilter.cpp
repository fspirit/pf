//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#include <iostream>
#include "ParticleFilter.h"

const float dt = 0.1;
const double Eps = 0.000001;

void ParticleFilter::Update(GpsMeasurement gps,  ControlMeasurement u, std::vector<LandmarkObservation> z)
{
    if (!isInitialised)
        InitialiseWithGpsMeasurement(gps);
    else
        Predict(u);

    UpdateWeights(z);
    Resample();
}

Particle ParticleFilter::GetBestParticle()
{
    auto p = std::max_element(particles.begin(), particles.end(),
                              [] (Particle const& lhs, Particle const& rhs) {
                                    return lhs.Weight < rhs.Weight;
                              });

    return *p;
}

void ParticleFilter::InitialiseWithGpsMeasurement(GpsMeasurement measurement)
{
    std::normal_distribution<float> xGauss(measurement.X, sqrt(gpsMeasurementNoise.XVariance));
    std::normal_distribution<float> yGauss(measurement.Y, sqrt(gpsMeasurementNoise.YVariance));
    std::normal_distribution<float> thetaGauss(measurement.Theta, sqrt(gpsMeasurementNoise.ThetaVariance));

    for (int i = 0; i != numberOfParticles; i++)
        particles.emplace_back(xGauss(randomEngine), yGauss(randomEngine), thetaGauss(randomEngine));

    isInitialised = true;
}

void ParticleFilter::Predict(ControlMeasurement u)
{
    for (auto& p : particles)
    {
        if (fabs(u.YawRate) > Eps)
        {
            auto newTheta = p.Theta + u.YawRate * dt;
            p.X += u.Velocity / u.YawRate * ( sin(newTheta) - sin(p.Theta) );
            p.Y += u.Velocity / u.YawRate * ( - cos(newTheta) + cos(p.Theta) );
//            p.Theta = fmod(newTheta, 2 * M_PI);
            p.Theta = newTheta;
        }
        else
        {
            p.X += u.Velocity * cos(p.Theta) * dt;
            p.Y += u.Velocity * sin(p.Theta) * dt;
        }

        std::normal_distribution<float> xGauss(p.X, sqrt(gpsMeasurementNoise.XVariance));
        std::normal_distribution<float> yGauss(p.Y, sqrt(gpsMeasurementNoise.YVariance));
        std::normal_distribution<float> thetaGauss(p.Theta, sqrt(gpsMeasurementNoise.ThetaVariance));

        p.X = xGauss(randomEngine);
        p.Y = yGauss(randomEngine);
        p.Theta = thetaGauss(randomEngine);
    }
}

double ParticleFilter::GetDistance(LandmarkObservation const& observation, Landmark const& landmark)
{
    return sqrt((landmark.GetX() - observation.X) * (landmark.GetX() - observation.X) +
                (landmark.GetY() - observation.Y) * (landmark.GetY() - observation.Y));
}

void ParticleFilter::Resample()
{
    std::vector<Particle> resampledParticles;

    std::vector<float> weights(particles.size());
    std::transform(particles.begin(), particles.end(), weights.begin(), [](Particle& p) {
        return p.Weight;
    });

    std::discrete_distribution<float> d(weights.begin(), weights.end());

    for (int i = 0; i != particles.size(); i++)
    {
        auto p = particles[d(randomEngine)];
        resampledParticles.push_back(p);
    }

//    std::uniform_int_distribution<int> initIndexUD(0, particles.size() - 1);
//    std::uniform_real_distribution<double> betaStepUD(0.0, 1.0);
//
//    auto index = initIndexUD(randomEngine);
//    auto beta = 0.0;
//    double maxWeight = GetBestParticle().Weight;
//
//    for (int i = 0; i < particles.size(); i++)
//    {
//        beta += betaStepUD(randomEngine) * 2.0 * maxWeight;
//        while (index < beta)
//        {
//            beta -= particles[index].Weight;
//            index = (index + 1) % particles.size();
//        }
//        resampledParticles.push_back(particles[index]);
//    }

    particles = resampledParticles;
}

void ParticleFilter::UpdateWeights(std::vector<LandmarkObservation> &z)
{
    for (auto& p : particles)
    {
        std::vector<LandmarkObservation> observations = z;

        TransformCoordinates(p, observations);

        MapObservationsToLandmarks(observations);

        p.Weight = CalculateObservationsProbability(observations);

        p.LandmarkAssosiations = observations;
    }
}

void ParticleFilter::MapObservationsToLandmarks(std::vector<LandmarkObservation> &observations) const
{
    auto ls = landmarks;
    std::for_each(observations.begin(), observations.end(), [&ls](LandmarkObservation& obs) {

            sort(ls.begin(), ls.end(), [&obs](Landmark& lhs, Landmark& rhs){
                return GetDistance(obs, lhs) < GetDistance(obs, rhs);
            });

            obs.Id = ls.front().GetId();
        });
}

void ParticleFilter::TransformCoordinates(Particle &p,
                                          std::vector<LandmarkObservation> &observations) const
{
    std::for_each(observations.begin(), observations.end(), [&p](LandmarkObservation& obs) {
        LandmarkObservation t = obs;
        obs.Y = p.Y - sin(-p.Theta) * t.X + cos(-p.Theta) * t.Y;
        obs.X = p.X + cos(-p.Theta) * t.X + sin(-p.Theta) * t.Y;
    });
}

float ParticleFilter::CalculateObservationsProbability(std::vector<LandmarkObservation> &observations)
{
    double p = 1.0;
    std::for_each(observations.begin(), observations.end(), [this, &p](LandmarkObservation& obs) {
        auto lm = std::find_if(landmarks.begin(), landmarks.end(), [&obs](Landmark& l){
            return l.GetId() == obs.Id;
        });
        p *=  exp(- ( pow(obs.X - lm->GetX(), 2.) / (2. * landmarkObservationNoise.XVar) +
                        pow(obs.Y - lm->GetY(), 2.) / (2. * landmarkObservationNoise.YVar) )) /
                    (2. * M_PI * sqrt(landmarkObservationNoise.XVar) * sqrt(landmarkObservationNoise.YVar));
    });

    return p;
}

void ParticleFilter::TestPointConversion()
{
    Particle p { 4, 5, -M_PI / 2.0 };

    LandmarkObservation ob1 { 2, 2 };
    LandmarkObservation ob2 { 3, -2 };
    LandmarkObservation ob3 { 0, -4 };

    std::vector<LandmarkObservation> obs { ob1, ob2, ob3 };
    TransformCoordinates(p, obs);

    for (auto& ob : obs)
        std::cout << ob.X << " " << ob.Y << std::endl;
}

void ParticleFilter::TestPointsAssociation()
{
    Landmark l1 { 0, 5, 3 };
    Landmark l2 { 1, 2, 1 };
    Landmark l3 { 2, 6, 1 };
    Landmark l4 { 3, 7, 4 };
    Landmark l5 { 4, 4, 7 };

    std::vector<Landmark> ls {l1, l2, l3, l4, l5};
    landmarks = ls;

    LandmarkObservation ob1 { 6, 3 };
    LandmarkObservation ob2 { 2, 2 };
    LandmarkObservation ob3 { 0, 5 };

    std::vector<LandmarkObservation> obs { ob1, ob2, ob3 };

    MapObservationsToLandmarks(obs);

    for (auto& ob : obs)
        std::cout << ob.Id << std::endl;
}

void ParticleFilter::TestWeightsCalculation()
{
    Landmark l1 { 0, 5, 3 };
    Landmark l2 { 1, 2, 1 };
    Landmark l3 { 2, 6, 1 };
    Landmark l4 { 3, 7, 4 };
    Landmark l5 { 4, 4, 7 };

    std::vector<Landmark> ls {l1, l2, l3, l4, l5};
    landmarks = ls;

    LandmarkObservation ob1 { 0, 6, 3 };
    LandmarkObservation ob2 { 1, 2, 2 };
    LandmarkObservation ob3 { 1, 0, 5 };

    std::vector<LandmarkObservation> obs { ob1, ob2, ob3 };

    for (auto& ob : obs)
    {
        std::vector<LandmarkObservation> obs_ { ob };
        std::cout << CalculateObservationsProbability(obs_) << std::endl;
    }
}
