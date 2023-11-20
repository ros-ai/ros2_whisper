from setuptools import find_packages, setup

package_name = "audio_listener"

setup(
    name=package_name,
    version="1.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/audio_listener.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="m.huber_1994@hotmail.de",
    description="Audio common replica.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "audio_listener = audio_listener.audio_listener:main",
        ],
    },
)
