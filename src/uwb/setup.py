from setuptools import find_packages, setup

package_name = 'uwb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='wengkunduo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver = uwb.uwb_receiver:main',
            'filter = uwb.uwb_filter:main',
            'localization = uwb.uwb_localization:main',
            'last_filter = uwb.uwb_last_filter:main',
            'visualization = uwb.uwb_visualization:main',
            'position_nls = uwb.position_nls:main',
        ],
    },
)
