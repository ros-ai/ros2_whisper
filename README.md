# ROS 2 Whisper
ROS 2 inference for [whisper.cpp](https://github.com/ggerganov/whisper.cpp).

## Build
- Install `pyaudio`, see [install instructions](https://pypi.org/project/PyAudio/).
- Build this repository, do
```shell
mkdir -p ros-ai/src && cd ros-ai/src && \
git clone https://github.com/ros-ai/ros2_whisper.git && cd .. && \
colcon build --symlink-install --cmake-args -DWHISPER_CUDA=On --no-warn-unused-cli
```

## Demos
Run the inference action server (this will download models to `$HOME/.cache/whisper.cpp`):
```shell
ros2 launch whisper_bringup bringup.launch.py
```
Run a client node (activated on space bar press):
```shell
ros2 run whisper_demos whisper_on_key
```

Configure `whisper` parameters in [whisper.yaml](whisper_server/config//whisper.yaml).

## Available Actions
Action server under topic `inference` of type [Inference.action](whisper_msgs/action/Inference.action).

## Troubleshoot
- Encoder inference time: https://github.com/ggerganov/whisper.cpp/issues/10#issuecomment-1302462960
- Compile with GPU support (might differ between platforms): https://github.com/ggerganov/whisper.cpp#nvidia-gpu-support-via-cublas WHISPER_CUBLAS=On
