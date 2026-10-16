# Serial Communication Library

## About Protofiles

When updating `.proto` files, you need to force CMake to run again with the
following command:
```shell
colcon build --cmake-force-configure
```
Not doing so might result in the code not using the updated protofiles.

## Dependencies

```shell
sudo apt install libsdl2-dev libsdl2-mixer-dev
```
## ROS2 Services

Enter Stand-by mode:
- name: `/enter_stand_by`
- type: `std_srvs::srv::Trigger`
- example: `ros2 service call /enter_stand_by std_srvs/srv/Trigger`

Software emergency stop:
- name: `/emergency_stop`
- type: `std_srvs::srv::SetBool`
- example: `ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: True}"`

Reboot ECU:
- name: `/reboot_ecu`
- type: `std_srvs::srv::Trigger`
- example: `ros2 service call /reboot_ecu std_srvs/srv/Trigger`

Toggle Docking mode:
- name: `/toggle_docking_mode`
- type: `std_srvs::srv::SetBool`
- example: `ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: True}"`

Acknowledge an ECU fault:
- name: `/ack_fault`
- type: `std_srvs::srv::Trigger`
- `success=false` ("Another action request is running!") while another ECU
  action is queued
- example: `ros2 service call /ack_fault std_srvs/srv/Trigger`

Set the gains of one control loop:
- name: `/set_pid`
- type: `sd_msgs::srv::SetBasePid` (`loop`: 1 linear, 2 angular, 3 left wheel,
  4 right wheel; `config`: `k_p`, `k_i`, `k_d`, `enabled`)
- `success=false` for an unknown loop or while the ECU is not connected
- example: `ros2 service call /set_pid sd_msgs/srv/SetBasePid "{loop: 1, config: {k_p: 1.0, k_i: 0.1, k_d: 0.0, enabled: true}}"`

## ROS2 Topics (besides the usual `robot_base/*`)

- `robot_base/driver` (`sd_msgs/DriverStatus`): temperature of the motor
  driver and of each motor, current of each motor, bus voltage.
- `robot_base/pid` (`sd_msgs/BasePid`, transient local): the gains the ECU
  last reported for the 4 loops.
- `robot_base/rtos_tasks` (`sd_msgs/RtosTasks`): the ECU's FreeRTOS tasks,
  busiest first.
- `robot_base/params` (`sd_msgs/BaseParams`) also carries the ECU, driver
  and motor firmware versions, the packet rate and the good/bad packet
  counts.

## Parameters

- `mute_sounds` (bool, default `true`): the driver's own sound effects. It
  can be changed at runtime (`ros2 param set /robot_comm_serial mute_sounds
  false`); the web page's switch sets it too.

## Web page (bench only)

The page this node serves (websocketpp, `web/`) talks to the driver
directly, with no credential, scope or audit. On a robot use
**sd_supervisor**, which reaches the same functions through
robot_control_api and the ROM: Status → Periféricos (base details, reboot
ECU, acknowledge fault, LED test), Configurações → Base (PID, sounds) and
Tempo real → Base (velocities, IMU, RTOS tasks, ECU log). The page's limits,
Kalman and open loop forms are only `TODO`s in this driver.

## Tests

```shell
colcon build --packages-select robot_comm_serial
colcon test --packages-select robot_comm_serial
```

`pid_config_test`: each loop number points at its gains, an unknown loop is
null, a blank field keeps the current gain.

## Running as Systemd service

1. Create symlinks
```shell
# Service symlink
cd /etc/systemd/service
ln -s ~/ws_socialdroids/src/socialdroids/sd-robot-comm-serial/scripts/robot_comm_serial@.service .

# Launch symlink
cd /usr/local/bin
sudo ln -s ~/ws_socialdroids/src/socialdroids/sd-robot-comm-serial/scripts/run_as_service.sh .
```

2. Enable systemd service
```shell
sudo systemctl daemon-reload
sudo systemctl enable robot_comm_serial@<USER>.service
sudo systemctl start robot_comm_serial@<USER>.service
```

3. Check status
```shell
systemctl status robot_comm_serial@<USER>.service
```
> [!NOTE]
> `<USER>` should be the computer's username, e.g. `socialdroids`.


## Serial Library Info

[![Build Status](https://travis-ci.org/wjwwood/serial.svg?branch=master)](https://travis-ci.org/wjwwood/serial)*(Linux and OS X)* [![Build Status](https://ci.appveyor.com/api/projects/status/github/wjwwood/serial)](https://ci.appveyor.com/project/wjwwood/serial)*(Windows)*

This is a cross-platform library for interfacing with rs-232 serial like ports written in C++. It provides a modern C++ interface with a workflow designed to look and feel like PySerial, but with the speed and control provided by C++. 

This library is in use in several robotics related projects and can be built and installed to the OS like most unix libraries with make and then sudo make install, but because it is a catkin project it can also be built along side other catkin projects in a catkin workspace.

Serial is a class that provides the basic interface common to serial libraries (open, close, read, write, etc..) and requires no extra dependencies. It also provides tight control over timeouts and control over handshaking lines. 

### Documentation

Website: http://wjwwood.github.com/serial/

API Documentation: http://wjwwood.github.com/serial/doc/1.1.0/index.html
