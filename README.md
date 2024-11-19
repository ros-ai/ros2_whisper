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
Configure `whisper` parameters in [whisper.yaml](whisper_server/config/whisper.yaml).

### Whisper On Key

Run the inference action server (this will download models to `$HOME/.cache/whisper.cpp`):

```shell
ros2 launch whisper_bringup bringup.launch.py
```
Run a client node (activated on space bar press):

```shell
ros2 run whisper_demos whisper_on_key
```

### Stream

Bringup whisper:

```shell
ros2 launch whisper_bringup bringup.launch.py
```

Launch the live transcription stream:

```shell
ros2 run whisper_demos stream
```

## Parameters

To enable/disable inference, you can set the active parameter from the command line with:

```shell
ros2 param set /whisper/inference active false # false/true
```

- Audio will still be saved in the buffer but whisper will not be run.

## Available Actions

Action server under topic `inference` of type [Inference.action](whisper_idl/action/Inference.action).

- The feedback message regularly publishes the actively changing portion of the transcript.  

- The final result contains stale and active portions from the start of the inference.

## Published Topics

Topics of type [AudioTranscript.msg](whisper_idl/msg/AudioTranscript.msg) on `/whisper/transcript_stream`, which contain the entire transcript (stale and active), are published on updates to the transcript.  

Internally, the topic `/whisper/tokens` of type [WhisperTokens.msg](whisper_idl/msg/WhisperTokens.msg) is used to transfer the model output between nodes.

## Troubleshoot

- Encoder inference time: https://github.com/ggerganov/whisper.cpp/issues/10#issuecomment-1302462960
