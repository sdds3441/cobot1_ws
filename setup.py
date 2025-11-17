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
            'move_basic = dsr_rokey2.move_basic:main',
            'move_periodic = dsr_rokey2.move_periodic:main',
            'grip_test = dsr_rokey2.grip_test:main',
            'force_test = dsr_rokey2.force_test:main',
            'grip_tool = dsr_rokey2.grip_tool:main',
            'home = dsr_rokey2.home:main',
            'up = dsr_rokey2.up:main',
        ],
    },
)
