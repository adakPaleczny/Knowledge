FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN sudo apt update -y && sudo apt upgrade -y && apt-get install -y sudo &&\
     sudo apt install ros-noetic-roslaunch 

# Add user
RUN useradd -ms /bin/bash RPI && usermod -aG sudo RPI && echo 'RPI     ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER RPI 
WORKDIR /home/RPI/ros_ws

# Previlage
RUN sudo chown -R RPI /home/RPI 
RUN mkdir src

# Install git
RUN sudo apt-get install -y git

# Clone git ntpd driver
RUN cd ./src && git clone https://github.com/vooon/ntpd_driver.git

#Copy local packages
RUN mkdir src/ublox
COPY ublox src/ublox
COPY script.sh src/

RUN rosdep update &&\
    rosdep install --from-paths src --ignore-src -r -y

# Catkin_make
RUN source /opt/ros/noetic/setup.bash &&\
    cd /home/RPI/ros_ws && catkin_make

# Source
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/RPI/.bashrc && \
    echo "source /home/RPI/ros_ws/devel/setup.bash" >> /home/RPI/.bashrc && \
    source /home/RPI/.bashrc 

#RUN roslaunch
CMD ./src/script.sh
# CMD ["roslaunch", "ublox_gps ublox_device.launch"]
