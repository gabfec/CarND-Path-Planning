#include <uWS/uWS.h>
#include <string>
#include <vector>
#include "json.hpp"
#include "path_planner.h"

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {

  uWS::Hub h;

  PathPlanner pathPlanner("../data/highway_map.csv");

  h.onMessage([&pathPlanner]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);

      if (s != "")
      {
        auto j = nlohmann::json::parse(s);
        
        auto event = j[0].get<std::string>();
        
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          auto newPath = pathPlanner.ProcessTelemetry(j[1]);

          auto msg = "42[\"control\","+ newPath.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } 
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  constexpr auto port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}