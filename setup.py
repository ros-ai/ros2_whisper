from setuptools import setup

package_name = "ros2_whisper"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="m.huber_1994@hotmail.de",
    description="hisper package for ros2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "audio_command_node = ros2_whisper.audio_command_node:main",
            "whisper_inference_node = ros2_whisper.whisper_inference_node:main",
        ],
    },
)
