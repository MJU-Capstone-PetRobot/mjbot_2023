#!/bin/bash
set -e

vcs import < src/ros2.repos src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y

sudo apt-get install -y python3 python3-venv python3-pyaudio libportaudio2 libportaudiocpp0 portaudio19-dev libatlas-base-dev ffmpeg flac sense-hat
