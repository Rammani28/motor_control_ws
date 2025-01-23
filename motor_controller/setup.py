from setuptools import find_packages, setup

package_name = 'motor_controller'

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
    maintainer='rammani',
    maintainer_email='rammani@abc.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blink_led_jj = motor_controller.blink_node:main',
            'led_control_jj = motor_controller.led_controller:main',
            'led_subscriber_jj = motor_controller.Led_controller_follower:main'
        ],
    },
)
