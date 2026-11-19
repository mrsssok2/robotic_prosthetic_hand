from setuptools import find_packages, setup

package_name = "hand_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/web", ["web/index.html"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="S. Mahadev",
    maintainer_email="mrsssok222@gmail.com",
    description="MediaPipe hand tracking: proportional finger curl and gestures.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "tracker_node = hand_tracker.tracker_node:main",
        "ws_bridge = hand_tracker.ws_bridge:main",
        "fake_hand = hand_tracker.fake_hand:main",
        "udp_bridge = hand_tracker.udp_bridge:main",
    ]},
)
