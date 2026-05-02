#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class Plotter
{
public:
  Plotter();
  Plotter(std::string host, uint16_t port);
  static Plotter from_config(const std::string & config_path);

  ~Plotter();

  void plot(const nlohmann::json & json);

private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP
