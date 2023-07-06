from setuptools import setup

package_name = 'lane_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pradeshi',
    maintainer_email='pradeshi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "webcam_pub= lane_detector.camera_publisher:main",
            "opencv_sub= lane_detector.opencv_subscriber:main",
            "twist_sub= lane_detector.motor_controller:main",


        ],
    },
)
