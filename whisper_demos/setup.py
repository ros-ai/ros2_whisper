from setuptools import find_packages, setup

package_name = "whisper_demos"

setup(
    name=package_name,
    version="1.4.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="m.huber_1994@hotmail.de",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "whisper_on_key = whisper_demos.whisper_on_key:main",
            "stream = whisper_demos.stream:main",
        ],
    },
)
