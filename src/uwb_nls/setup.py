from setuptools import find_packages, setup

package_name = 'uwb_nls'

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
    maintainer='work',
    maintainer_email='work@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver = uwb_nls.uwb_receiver:main',
            #'filter_ransac = uwb_nls.uwb_ransac_filter:main',
            'first_filter = uwb_nls.uwb_filter:main',
            'position_nls = uwb_nls.position_nls:main',
            'last_filter_nls = uwb_nls.uwb_last_filter_nls:main',
            'visualization_all = uwb_nls.uwb_visualization:main',
        ],
    },
)
