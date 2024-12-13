^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ROS 2 Whisper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2024-12-11)
------------------

* `whisper_cpp_vendor`: `whisper.cpp` 1.6.2 to 1.7.2 release, build changes
* Added live audio transcription streaming
* `whisper_server` changes:
   * Holding incoming Audio data in a Ring Buffer (removed BatchBuffer, drop oldest audio).
   * Transcribing the entire buffer of audio data with whisper.cpp on a timer interrupt
   * Publishing the resulting tokens + probabilities on topic `/whisper/tokens`
   * Removing the Action Server
   * New Node Parameters:
       * `active` -- Boolean to control if whisper.cpp should be run or not.
       * `callback_ms` -- Integer controlling how often whisper.cpp is called.
       * `buffer_capacity` -- Integer number of seconds previous where audio is transcribed.
* `transcript_manager` package added to:
   * Store record of what was previously transcribed.
   * Track what is currently being transcribed. Align and update the text from subscribed topic `/whisper/tokens`.
       * Updates done on timer interrupt
   * Host the Action Server which was previously part of `whisper_server`
   * Publish the entire transcript (previous and current) under `/whisper/transcript_stream`
       * Published transcript contains text and estimated segment markings, segment timestamps
* `whisper_demos`: Add `stream` node
* `whisper_idl`:
   * Added `msg/WhisperTokens.msg`, `msg/AudioTranscript.msg`
   * Added `launch/replay.launch.py` which does not bring up `audio_listener`
* `whisper_util`: Changes to directly inference and then serialize whisper.cpp model output, also containing probability data.


1.3.1 (2024-07-01)
------------------

* `whisper_msgs`: Changed to `whisper_idl` package
* `whisper_bringup`: Changed executor to `MultiThreadedExecutor` so audio and inference can run in parallel on `whisper_server`

1.3.0 (2024-06-21)
------------------
* `whisper_cpp_vendor`: `whisper.cpp` 1.5.4 to 1.6.2 release
* `whisper_cpp_vendor`: CMake build flag `WHISPER_CUBLAS` to `WHISPER_CUDA`
* `whisper_server`: Removed launch mixins
* `whisper_server`: Updated parameters for `whisper.yaml`
* `whisper_server`: Fixed `whisper` initialization order

1.2.1 (2023-12-15)
------------------
* `whisper_cpp_vendor`: Set C++ standard to C++11 for target

1.2.0 (2023-11-19)
------------------
* `whisper_util`: Upgrade to `whisper.cpp` 1.5.0 release https://github.com/ggerganov/whisper.cpp/releases/tag/v1.5.0 (full CUDA backend)

1.1.0 (2023-09-01)
------------------
* `whisper_demos`: Improved terminal output
* `whisper_server`: Improved state machine

1.0.0 (2023-08-31)
------------------
* Initial release
* Uses whisper.cpp https://github.com/ggerganov/whisper.cpp instead of OpenAI Whisper Python library
* Introduces `whisper_bringup` package for launch
* Introduces `whisper_cpp_vendor` to expose whisper.cpp as a ROS 2 package
* Introduces `whisper_demos` for exemplary usage of `whisper_server`
* Introduces `whisper_msgs` for ROS 2 message definitions
* Introduces `whisper_server` for whisper action server
* Introduces `whisper_utils` package for inference utilities

alpha 0.1.0 (2023-08-15)
------------------------
* Alpha release
* Uses OpenAI Whisper Python library https://github.com/openai/whisper
