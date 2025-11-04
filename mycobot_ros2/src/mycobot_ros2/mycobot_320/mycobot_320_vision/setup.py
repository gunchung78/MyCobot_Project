from setuptools import find_packages, setup

package_name = 'mycobot_320_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/box_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geon',
    maintainer_email='gunchung78@gmail.com',
    description='myCobot vision: publish (u,v,roll) from camera',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # ⬇️ 여기!  mycobot_320_vision.box_detector_node:main  로 써야 함
            'box_detector = mycobot_320_vision.box_detector_node:main',
        ],
    },
)
