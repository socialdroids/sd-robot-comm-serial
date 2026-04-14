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
