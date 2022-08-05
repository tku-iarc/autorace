# Install Step

## Environment

- Ubuntu 18.04 Server
  - ROS Melodic
  - Python 2.7.17
  - Tensorflow 1.15
  - Scikit-fuzzy==0.4.2(to be confirmed)

## Robot Computer Install Step

  You can choose one of the two, Docker version is recommended

### Docker Version

  1. install Docekr
      Reference: [Docker offcial website](https://docs.docker.com/engine/install/ubuntu/)
      Note: **Post-installation step for linux** steps need to be completed

  2. Create AutoRace workspace and Go to AutoRace workspace

      ```shell
        $ mkdir -p autorace_ws/src && cd autorace_ws/src
      ```

  3. Download source code

      ```shell
        $ git clone xxx .
      ```

  4. Setup USB driver rules

      ```shell
        $ ./usb_rules/usb_setup.sh
        # input Admin password
      ```

  5. Go to AutoRace workspace's docker file

      ```shell
        $ cd docker
      ```

  6. Run build Docker image script

      ```shell
        $ ./build.sh
      ```

  7. Run Docker container script

      ```shell
        $ ./run.sh
      ```

### Full-install Version

  1. Install ROS Melodic
      Reference: [ROS Wiki](http://wiki.ros.org/melodic/Installation/Ubuntu)

  2. Install catkin Dependent package

      ```shell
        $ sudo apt install g++
      ```

  3. Install Turtlebot3 dependent ROS package

      ```shell
        $ sudo apt update && sudo apt install -y \
            ros-melodic-hls-lfcd-lds-driver \
            ros-melodic-dynamixel-sdk \
            ros-melodic-turtlebot2 \
            ros-melodic-turtlebot3-msgs \
            ros-melodic-joy \
            ros-melodic-teleop-twist-joy \
            ros-melodic-teleop-twist-keyboard \
            ros-melodic-laser-proc \
            ros-melodic-rgbd-launch \
            ros-melodic-depthimage-to-laserscan \
            ros-melodic-camera-info-manager \
            ros-melodic-rosserial-arduino \
            ros-melodic-rosserial-python \
            ros-melodic-rosserial-server \
            ros-melodic-rosserial-client \
            ros-melodic-rosserial-msgs \
            ros-melodic-amcl \
            ros-melodic-map-server \
            ros-melodic-move-base \
            ros-melodic-compressed-image-transport \
            ros-melodic-gmapping \
            ros-melodic-navigation \
            ros-melodic-interactive-markers
      ```

  4. Install Turtlebot3 Package

      ```shell
        $ sudo apt install -y \
            ros-melodic-dynamixel-sdk \
            ros-melodic-turtlebot3-msgs \
            ros-melodic-turtlebot3
      ```

  5. Setup Turtlebot3 Model name

      ```shell
        $ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
      ```

## Remote User Computer

 1. Setup OpenCR
    Reference: [OpenCR Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup)

# RUN AutoRace Step

  1. Setup AutoRace parameters
  2. Run AutoRace Main launch

