from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package indexing
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安裝 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安裝 launch 檔案
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]'))),
        # 安裝 config 資料夾中的 JSON 檔案
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anderson',
    maintainer_email='anderson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_dual = control_motor.control_dual:main',
            'Twist2Speed = control_motor.Twist2Speed:main',
            'msg_choose = control_motor.msg_choose:main',
            'rc_receiver = control_motor.rc_receiver:main',
            'test1 = control_motor.test1:main'
        ],
    },
)
