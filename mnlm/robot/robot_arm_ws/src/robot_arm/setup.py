import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_arm"

# Create an empty marker file for the ament index
ament_index_path = os.path.join("share", "ament_index", "resource_index", "packages")
os.makedirs(ament_index_path, exist_ok=True)
with open(os.path.join(ament_index_path, package_name), "w") as f:
    pass

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    # Non-code files to include in the package
    data_files=[
        # Register your package with the ament index (so ROS tools can find it)
        (ament_index_path, [os.path.join(ament_index_path, package_name)]),
        # Include your package.xml file
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files from the 'launch' directory
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        # Include all SDF model files from the 'models' directory
        (os.path.join("share", package_name, "models"), glob("models/*")),
        # Include config files
        (os.path.join("share", package_name, "config"), glob("config/*")),
        # Include rviz files
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools", "flask"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="2237303+yxjiang@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_receiver_node = robot_arm.command_receiver:main",
            "command_dispatcher_node = robot_arm.command_dispatcher:main",
            "joint_move_node = robot_arm.joint_move:main",
        ],
    },
)
