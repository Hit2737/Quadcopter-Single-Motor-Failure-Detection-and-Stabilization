from setuptools import find_packages, setup

package_name = 'px4_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'numpy',
        'torch',
        'pandas',
        'scikit-learn',
    ],
    zip_safe=True,
    maintainer='demolus',
    maintainer_email='parthgovale@gmail.com',
    description='Package for PX4 model including data processing and reinforcement learning agent.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'detector = px4_detector.detector:main',
            'trajectory_publisher = px4_detector.trajectory_publisher:main',
        ],
    },
)