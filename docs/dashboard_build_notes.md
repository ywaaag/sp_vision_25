# Dashboard Build Notes

This document records the Dashboard MQTT backend boundary that remains in the vision repository.

## CMake Integration

The root `CMakeLists.txt` includes `cmake/DashboardDeps.cmake`.

- `find_package(PahoMqttCpp QUIET)` probes optional C++ MQTT support.
- If Paho MQTT C++ is missing, normal non-dashboard vision targets still configure without the dashboard MQTT path.
- When `mqtt_bridge` exists, only `auto_aim_debug_mpc` links `mqtt_bridge` and `dashboard_params`, and only `auto_aim_debug_mpc` defines `SP_VISION_ENABLE_DASHBOARD_MQTT`.
- `standard_mpc` remains upstream behavior.

## Vision App Dependencies Added By Dashboard

Install these only for the Dashboard MQTT backend feature:

```bash
sudo apt update
sudo apt install -y libpaho-mqtt-dev libpaho-mqttpp-dev nlohmann-json3-dev
```

If the image already provides `nlohmann/json.hpp`, `nlohmann-json3-dev` is not an additional requirement.

## Runtime Boundary

Start `auto_aim_debug_mpc` manually with real hardware access when Dashboard is needed. Dashboard settings are read from `configs/standard3.yaml`:

```yaml
dashboard:
  enabled: false
  robot_id: "myrobot"
  mqtt_host: "tcp://127.0.0.1:1883"
```

Default startup without Dashboard remains unchanged:

```bash
./build/standard_mpc configs/standard3.yaml
```

or:

```bash
./build/auto_aim_debug_mpc configs/standard3.yaml
```

Dashboard startup uses `auto_aim_debug_mpc`:

```bash
./build/auto_aim_debug_mpc --dashboard --mqtt-host tcp://127.0.0.1:1883 configs/standard3.yaml
```

If the broker is on another LAN host, use that host address:

```bash
./build/auto_aim_debug_mpc --dashboard --mqtt-host tcp://Dashboard主机IP:1883 configs/standard3.yaml
```

CLI options override YAML. `--dashboard` forces enable, `--robot-id` overrides `dashboard.robot_id`, and `--mqtt-host` overrides `dashboard.mqtt_host`.

Do not run these commands on a machine without camera, gimbal, CAN, and model assets.

## Frontend Split

The Dashboard browser client and service stack have moved to an independent frontend repository. This repository keeps only the C++ MQTT backend and protocol documentation.
