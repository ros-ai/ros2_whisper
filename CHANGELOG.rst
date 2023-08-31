^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ROS 2 Whisper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.0.0 (2023-08-31)
------------------
* Initial release
* Uses whisper.cpp https://github.com/ggerganov/whisper.cpp instead of OpenAI Whisper Python library
* Introduces `whisper_bringup` package for launch
* Introduces `whisper_cpp_vendor` to expose whisper.cpp as a ROS 2 package
* Introduces `whisper_demos` for exemplary usage of `whisper_nodes`
* Introduces `whisper_msgs` for ROS 2 message definitions
* Introduces `whisper_nodes` for whisper action server
* Introduces `whisper_utils` package for inference utilities

alpha 0.1.0 (2023-08-15)
------------------------
* Alpha release
* Uses OpenAI Whisper Python library https://github.com/openai/whisper
