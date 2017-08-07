//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#include "WebSocketMessageHandler.h"

string WebSocketMessageHandler::CreateResponseMessage(const Particle& bestParticle)
{
    json msgJson;

    msgJson["best_particle_x"] = bestParticle.X;
    msgJson["best_particle_y"] = bestParticle.Y;
    msgJson["best_particle_theta"] = bestParticle.Theta;

    FillLandmarkEstimates(bestParticle, msgJson);

    return "42[\"best_particle\"," + msgJson.dump() + "]";
}

void WebSocketMessageHandler::FillLandmarkEstimates(const Particle &bestParticle, json &msgJson) const
{
    if (bestParticle.LandmarkAssosiations.empty())
        return;

    std::vector<int> landmarkIds;
    std::vector<float> landmarkEstimatedXs;
    std::vector<float> landmarkEstimatedYs;

    for (auto& a : bestParticle.LandmarkAssosiations)
    {
        landmarkIds.push_back(a.Id);
        landmarkEstimatedXs.push_back(a.X);
        landmarkEstimatedYs.push_back(a.Y);
    }

    msgJson["best_particle_associations"] = CopyToString(landmarkIds);
    msgJson["best_particle_sense_x"] =  CopyToString(landmarkEstimatedXs);
    msgJson["best_particle_sense_y"] = CopyToString(landmarkEstimatedYs);
}

string WebSocketMessageHandler::ProcessMessageContent(string& content)
{
    auto jsonContent = json::parse(content);
    string eventType = jsonContent[0].get<string>();

    string response;

    if (eventType == "telemetry")
    {
        auto eventData = jsonContent[1];

        auto gpsMeasurement = ReadGpsMeasurementFromJson(eventData);
        auto controlMeasurement = ReadControlMeasurementFromJson(eventData);
        auto landmarkObservations = ReadLandmarkObservationsFromJson(eventData);

        particleFilter.Update(gpsMeasurement, controlMeasurement, landmarkObservations);
        auto p = particleFilter.GetBestParticle();

        response = CreateResponseMessage(p);
    }
    return response;
}

GpsMeasurement WebSocketMessageHandler::ReadGpsMeasurementFromJson(json data)
{
    auto x = std::stof(data["sense_x"].get<std::string>());
    auto y = std::stof(data["sense_y"].get<std::string>());
    auto theta = std::stof(data["sense_theta"].get<std::string>());

    return {x, y, theta};
}

ControlMeasurement WebSocketMessageHandler::ReadControlMeasurementFromJson(json data)
{
    auto velocity = std::stof(data["previous_velocity"].get<std::string>());
    auto yawRate = std::stof(data["previous_yawrate"].get<std::string>());

    return {velocity, yawRate};
}

std::vector<LandmarkObservation> WebSocketMessageHandler::ReadLandmarkObservationsFromJson(json data)
{
    std::vector<LandmarkObservation> observations;
    auto x = ParseVectorOfMeasurements(data, "sense_observations_x");
    auto y = ParseVectorOfMeasurements(data, "sense_observations_y");

    for (int i = 0; i < x.size(); i++)
        observations.emplace_back(x[i],  y[i]);

    return observations;
}

std::vector<float> WebSocketMessageHandler::ParseVectorOfMeasurements(const json &data, std::string key) const {

    string jsonString = data[key];
    std::istringstream dataStream(jsonString);

    std::vector<float> result;
    copy(std::istream_iterator<float>(dataStream),
         std::istream_iterator<float>(),
         std::back_inserter(result));
    return result;
}

string WebSocketMessageHandler::GetMessageContent(const string& message)
{
    string content;

    bool hasNullContent = (message.find("null") != string::npos);
    if (hasNullContent)
        return content;

    auto b1 = message.find_first_of('[');
    auto b2 = message.find_first_of(']');

    if (b1 != string::npos && b2 != string::npos)
        content = message.substr(b1, b2 - b1 + 1);

    return content;
}

bool WebSocketMessageHandler::MessageHasExpectedPrefix(const string& message)
{
    // "42" at the start of the message means there's a websocket message event.
    //
    const string prefix ("42");
    return (message.substr(0, prefix.size()) == prefix);
}

void WebSocketMessageHandler::HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws)
{
    if (!MessageHasExpectedPrefix(message))
    {
        SendDefaultResponse(ws);
        return;
    }

    auto content = GetMessageContent(message);
    if (content.empty())
    {
        SendDefaultResponse(ws);
        return;
    }

    auto response = ProcessMessageContent(content);

//    std::cout << content << std::endl;
//    std::cout << response << std::endl;

    if (!response.empty())
        ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
    else
        SendDefaultResponse(ws);
}

void WebSocketMessageHandler::SendDefaultResponse(uWS::WebSocket<uWS::SERVER>& ws) const
{
    string response = "42[\"manual\",{}]";
    ws.send(response.data(), response.length(), uWS::TEXT);
}

template <typename T>
string WebSocketMessageHandler::CopyToString(std::vector<T> data) const
{
    std::stringstream ss;
    std::copy(data.begin(), data.end(), std::ostream_iterator<T>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}


