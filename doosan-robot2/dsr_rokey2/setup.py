from setuptools import find_packages, setup

package_name = 'dsr_rokey2'

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
    maintainer='hyochan',
    maintainer_email='khcc0519@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sky_cleaner = dsr_rokey2.sky_cleaner:main',
            'move_periodic = dsr_rokey2.move_periodic:main',
            'grip_test = dsr_rokey2.grip_test:main',
            'force_test = dsr_rokey2.force_test:main',
            'grip_tool = dsr_rokey2.grip_tool:main',
            'home = dsr_rokey2.home:main',
            'up = dsr_rokey2.up:main',
            'cleanning_task = dsr_rokey2.cleanning_task:main',
            #'move_basic = dsr_rokey2.move_basic:main',
            #'wipe_action = dsr_rokey2.wriper:main',
            #'move_vertical = dsr_rokey2.move_vertical:main',
            'run_master_sequence = dsr_rokey2.master_runner:main',
            #'grip_tool = dsr_rokey2.grip_tool:main',
            'status_monitor = dsr_rokey2.status_monitor:main',
        ],
    },
)
