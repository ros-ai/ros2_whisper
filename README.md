# ROS 2 Whisper
ROS 2 inference for [whisper.cpp](https://github.com/ggerganov/whisper.cpp).

## Dependencies

## Build
- Install `pyaudio`, see [install instructions](https://pypi.org/project/PyAudio/).
- Build this repository, do
```shell
mkdir -p whisper_ws/src && cd whisper_ws/src && \
git clone https://github.com/ros-ai/ros2_whisper.git && cd .. && \
colcon build --symlink-install
```

## Demos
Run the inference nodes (this will download models to `$HOME/.cache/whisper.cpp`):
```shell
ros2 launch whisper_bringup bringup.launch.py n_thread:=8
```
Run a client node (activated on voice threshold):
```shell
ros2 run whisper_demos voice_threshold_node
```

## Troubleshoot
Encoder inference time: https://github.com/ggerganov/whisper.cpp/issues/10#issuecomment-1302462960
Compile with GPU support: https://github.com/ggerganov/whisper.cpp#nvidia-gpu-support-via-cublas WHISPER_CUBLAS = On
Quantize: 
