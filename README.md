# ROS2 Whisper
Repository showcases inference of the [OpenAI Whisper](https://github.com/openai/whisper) within a ROS2 node.

## Preparation
### Dependencies
This repository requires `pyaudio` and `whisper`. Install via respective install instructions
- `pyaudio` [install instructions](https://pypi.org/project/PyAudio/)
- `whisper` [install instructions](https://github.com/openai/whisper#setup)
### Build
To build, do

```shell
mkdir -p ros2_whisper_ws/src && cd ros2_whisper_ws/src && \
git clone https://github.com/mhubii/ros2_whisper.git && cd .. && \
colcon build && source install/setup.bash
```

## Run
To run, do
```shell
ros2 launch ros2_whisper ros2_whisper.launch.py
```
