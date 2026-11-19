from setuptools import find_packages, setup

package_name = "hand_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="S. Mahadev",
    maintainer_email="mrsssok222@gmail.com",
    description="Curl to servo angles and joint position commands.",
    license="MIT",
    entry_points={"console_scripts": ["controller_node = hand_controller.controller_node:main"]},
)
