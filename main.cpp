#include <uWS/uWS.h>
#include "json.hpp"

#include "WebSocketMessageHandler.h"

using std::string;
using std::cout;
using std::endl;
using std::cerr;

const float GpsMeasurementXVariance = 0.3 * 0.3;
const float GpsMeasurementYVariance = 0.3 * 0.3;
const float GpsMeasurementThetaVariance = 0.01 * 0.01;

const float LandmarkObservationXVariance = 0.3 * 0.3;
const float LandmarkObservationYVariance = 0.3 * 0.3;

const int NumberOfParticles = 5;

int main(int argc, char * argv[])
{
    uWS::Hub h;

    auto landmarks = Landmark::ReadLandmarksFromFile();
    if (landmarks.empty())
    {
        cerr << "Error: Could not open map file" << endl;
        return -1;
    }

    ParticleFilter pf(GpsMeasurementNoise(GpsMeasurementXVariance, GpsMeasurementYVariance, GpsMeasurementThetaVariance),
                      ObservationsNoise(LandmarkObservationXVariance, LandmarkObservationYVariance),
                      landmarks, NumberOfParticles);
    WebSocketMessageHandler handler(pf);

    h.onMessage([&handler](uWS::WebSocket<uWS::SERVER> ws,
                           char * data,
                           size_t length,
                           uWS::OpCode opCode)
                {
                    if (length == 0)
                        return;

                    string message (data, length);
                    handler.HandleMessage(message, ws);
                });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                   {
                       cout << "Connected" << endl;
                   });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char * message, size_t length)
                      {
                          ws.close();
                          cout << "Disconnected" << endl;
                      });

    const int port = 4567;
    if (h.listen(port))
        cout << "Listening to port " << port << endl;
    else
        cerr << "Failed to listen to port" << endl;

    h.run();

//    pf.TestPointConversion();
//    pf.TestPointsAssociation();
//    pf.TestWeightsCalculation();
}