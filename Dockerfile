FROM ros:humble

ENV ROS_HOME=/opt/ros_home

WORKDIR /tmp/maverick
COPY . .

RUN apt-get update \
 && xargs apt-get install -y < /tmp/maverick/tooling.apt \
 && curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin \
 && rosdep update \
 && rosdep install --from-paths /tmp/maverick --ignore-src -r -y \
 && rm -rf /tmp/maverick

WORKDIR /
