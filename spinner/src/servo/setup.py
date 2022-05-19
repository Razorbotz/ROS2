from setuptools import setup

package_name = 'servo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='andrewburroughs17@gmail.com',
    description='Servo package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'servo_node = servo.servo_node:main',
        ],
    },
)
