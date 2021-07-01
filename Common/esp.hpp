#pragma once

#include <cstdint>
#include <string_view>

#include "utility/debug.hpp"
#include "utility/log.hpp"
#include "peripherals/lpc40xx/uart.hpp"
#include "devices/communication/esp8266.hpp"

namespace sjsu::common
{
/// Esp class manages the esp01/esp8266 WiFi module on the rover
class Esp
{
 public:
  Esp()
      : esp_(sjsu::lpc40xx::GetUart<3>()),
        wifi_(esp_.GetWiFi()),
        socket_(esp_.GetInternetSocket()){};

  /// Initializes the Wi-Fi module by connecting to WiFi
  void Initialize()
  {
    sjsu::LogInfo("Initializing Wi-Fi module...");
    esp_.Initialize();
    ConnectToWiFi();
  };

  /// Sends a GET request to the specified url
  /// @param endpoint i.e. /endpoint?example=parameter
  /// @return the response body data
  std::string_view GET(std::string endpoint)
  {
    request_ = "GET /" + endpoint + " HTTP/1.1\r\nHost: " + url_.data() +
               "\r\nContent-Type: application/json\r\n\r\n";
    ConnectToServer();
    WriteRequest();
    ReadResponse();
    response_ = response_.substr(response_.find("\r\n\r\n"));
    auto body = response_.substr(response_.find("{"));
    puts(body.data());
    return body;
    // return GetJsonBodyFromResponse();
  };

 private:
  /// Attempts to connect to the hardcoded WiFi address
  void ConnectToWiFi()
  {
    while (true)
    {
      sjsu::LogInfo("Attempting to connect to %s...", kSsid);
      if (wifi_.ConnectToAccessPoint(kSsid, kPassword, kDefaultTimeout))
      {
        break;
      }
      sjsu::LogError("Failed to connect to %s... Retrying...", kSsid);
    }
    sjsu::LogInfo("Connected!");
  }

  void ConnectToServer()
  {
    sjsu::LogInfo("Connecting to %s...", url_.data());
    socket_.Connect(sjsu::InternetSocket::Protocol::kTCP, url_, kPort,
                    kDefaultTimeout);
  }

  void WriteRequest()
  {
    sjsu::LogInfo("Writing request to server...");
    std::span write_payload(reinterpret_cast<const uint8_t *>(request_.data()),
                            request_.size());
    socket_.Write(write_payload, kDefaultTimeout);
  }

  void ReadResponse()
  {
    sjsu::LogInfo("Reading back response from server...");
    std::array<uint8_t, 1024 * 2> raw;
    size_t res_size = socket_.Read(raw, kDefaultTimeout);
    std::string_view response_(reinterpret_cast<char *>(raw.data()), res_size);
  }

  /// Parses the GET response
  /// @return the response body
  std::string_view GetJsonBodyFromResponse()
  {
    response_ = response_.substr(response_.find("\r\n\r\n"));
    auto body = response_.substr(response_.find("{"));
    return body;
  }

  /// Checks Wi-Fi connection and will attempt to reconnect
  void isConnectedToWiFi()
  {
    if (!wifi_.IsConnected())  // TODO: Not yet implemented
    {
      sjsu::LogError("Lost connection to %s... Reconnecting...", kSsid);
      ConnectToWiFi();
    }
  };

  sjsu::Esp8266 esp_;
  sjsu::WiFi & wifi_;
  sjsu::InternetSocket & socket_;
  std::string_view request_;
  std::string_view response_;
  std::string_view url_  = "jsonplaceholder.typicode.com";
  const uint16_t kPort   = 80;
  const char * kSsid     = "GarzaLine";
  const char * kPassword = "NRG523509";
  const std::chrono::nanoseconds kDefaultTimeout = 10s;
};
}  // namespace sjsu::common
