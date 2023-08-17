# ROS 2 Whisper
Repository showcases inference of the [OpenAI Whisper](https://github.com/openai/whisper) within a ROS 2 node.

## Preparation
### Dependencies
This repository requires `pyaudio` and `whisper`. Install via respective install instructions
- `pyaudio` [install instructions](https://pypi.org/project/PyAudio/)
- `whisper` [install instructions](https://github.com/openai/whisper#setup)
### Build
To build, do

```shell
mkdir -p whisper_ws/src && cd whisper_ws/src && \
git clone https://github.com/mhubii/whisper.git && cd .. && \
colcon build --symlink-install
```

## Run
To run, do
```shell
source install/setup.bash && \
ros2 launch whisper whisper.launch.py
```

To print the inferenced text, do
```shell
ros2 topic echo /whisper_inference_node/text
```
