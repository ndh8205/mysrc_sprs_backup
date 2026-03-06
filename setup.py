import os
from setuptools import setup, find_packages

package_name = 'orbit_sim'

def package_files(directory):
    """
    Recursively finds all files in a directory and prepares them for setup.py data_files.
    This preserves the entire directory structure.
    """
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # Create a tuple of (destination_directory, [source_files])
            # The destination is relative to the package's share directory
            dest_dir = os.path.join('share', package_name, path)
            src_file = os.path.join(path, filename)
            paths.append((dest_dir, [src_file]))
    return paths

# Create the list of data_files by walking through the 'models', 'worlds', 'launch', and 'data' directories
data_files_list = package_files('models')
data_files_list.extend(package_files('worlds'))
data_files_list.extend(package_files('launch'))
data_files_list.extend(package_files('data'))
data_files_list.extend(package_files('config'))
data_files_list.extend(package_files('urdf'))

# Add package.xml and resource file
data_files_list.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files_list.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Integrated orbit simulation package for Space ROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_satellite_controller = orbit_sim.multi_satellite_controller:main',
            'multi_satellite_controller_service = orbit_sim.multi_satellite_controller_service:main',
            'multi_satellite_controller_guidence = orbit_sim.multi_satellite_controller_guidence:main',
            'aruco_detector = orbit_sim.aruco_detector:main',
            'aruco_simulator = orbit_sim.aruco_simulator:main',
            'gazebo_aruco_controller = orbit_sim.gazebo_aruco_controller:main',
            'imu_visualizer = orbit_sim.imu_visualizer:main',
            'torque_publisher = orbit_sim.torque_publisher_node:main',
            'gco_controller = orbit_sim.gco_controller:main',
            'force_torque_controller = orbit_sim.force_torque_controller:main',
            'orbit_LVLH_gco = orbit_sim.orbit_LVLH_gco:main',
            'pose_control_camtest = orbit_sim.pose_control_camtest:main',
            'lvlh_sim_node = orbit_sim.lvlh_sim_node:main',
            'lidar_bridge = orbit_sim.lidar_bridge:main',
            'laser_to_pointcloud = orbit_sim.laser_to_pointcloud:main',
            'lidar_orbit_scanner = orbit_sim.lidar_orbit_scanner:main',
            'pointcloud_mapper = orbit_sim.pointcloud_mapper:main',
            'impedance_controller = orbit_sim.impedance_controller:main',
            'dual_arm_impedance_controller = orbit_sim.dual_arm_impedance_controller:main',
            'http_ros_bridge = orbit_sim.http_ros_bridge:main',
            'cubesat_cmg_controller = orbit_sim.cubesat_cmg_controller:main',
            'cmg_testbed_commander = orbit_sim.cmg_testbed_commander:main',
            'cmg_testbed_scenario = orbit_sim.cmg_testbed_scenario:main',
            'cubesat_cmg_commander = orbit_sim.cubesat_cmg_commander:main',
            'stereo_test_gui = orbit_sim.stereo_test_gui:main',
            'mars_hexacopter_cmg_commander = orbit_sim.mars_hexacopter_cmg_commander:main',
        ],
    },
)