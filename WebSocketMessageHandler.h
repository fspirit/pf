//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#ifndef PF_WEBSOCKETMESSAGEHANDLER_H
#define PF_WEBSOCKETMESSAGEHANDLER_H

#include <string>

#include <uWS/uWS.h>
#include "json.hpp"
#include "ParticleFilter.h"


using std::string;
using json = nlohmann::json;

class WebSocketMessageHandler
{
public:
    WebSocketMessageHandler(const ParticleFilter &particleFilter) : particleFilter(particleFilter) {}
    void HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws);

private:
    ParticleFilter particleFilter;

    bool MessageHasExpectedPrefix(const string& message);
    string GetMessageContent(const string& message);
    string ProcessMessageContent(string& content);
    string CreateResponseMessage(const Particle& bestParticle);

    ControlMeasurement ReadControlMeasurementFromJson(json data);
    std::vector<LandmarkObservation> ReadLandmarkObservationsFromJson(json data);
    GpsMeasurement ReadGpsMeasurementFromJson(json data);
    std::vector<float> ParseVectorOfMeasurements(const json &data, std::string key) const;

    void FillLandmarkEstimates(const Particle &bestParticle, json &msgJson) const;
    void SendDefaultResponse(uWS::WebSocket<uWS::SERVER> &ws) const;

    template<typename T>
    std::string CopyToString(std::vector<T> vector) const;
};


#endif //PF_WEBSOCKETMESSAGEHANDLER_H
