#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace tools
{
namespace
{
constexpr char DEFAULT_PLOTTER_HOST[] = "127.0.0.1";
constexpr uint16_t DEFAULT_PLOTTER_PORT = 9870;
}  // namespace

Plotter::Plotter() : Plotter(DEFAULT_PLOTTER_HOST, DEFAULT_PLOTTER_PORT) {}

Plotter::Plotter(std::string host, uint16_t port)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port);
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

Plotter Plotter::from_config(const std::string & config_path)
{
  auto host = std::string(DEFAULT_PLOTTER_HOST);
  auto port = DEFAULT_PLOTTER_PORT;

  const auto yaml = tools::load(config_path);
  if (yaml["plotter"]) {
    const auto plotter = yaml["plotter"];
    if (plotter["host"] && !plotter["host"].IsNull() && plotter["host"].IsScalar()) {
      const auto configured_host = plotter["host"].as<std::string>();
      if (!configured_host.empty()) {
        host = configured_host;
      }
    }
    if (plotter["port"] && !plotter["port"].IsNull() && plotter["port"].IsScalar()) {
      port = plotter["port"].as<uint16_t>();
    }
  }

  tools::logger()->info("Plotter UDP target {}:{}", host, port);
  return Plotter(host, port);
}

Plotter::~Plotter() { ::close(socket_); }

void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

}  // namespace tools
