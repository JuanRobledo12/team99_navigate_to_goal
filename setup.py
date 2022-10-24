from setuptools import setup

package_name = 'team99_navigate_to_goal'

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
    maintainer='tony',
    maintainer_email='jarobledo98@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'print_fixed_odometry = team99_navigate_to_goal.print_fixed_odometry:main',
            'test_odom = team99_navigate_to_goal.test_odom:main',
        ],
    },
)
