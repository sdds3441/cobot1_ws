from setuptools import setup

package_name = 'ee_pose_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='EE pose transform publisher and subscriber package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ee_pose_from_tf = ee_pose_pkg.ee_pose_from_tf:main',
            'ee_pose_rpy_sub = ee_pose_pkg.ee_pose_rpy_sub:main',
        ],
    },
)
