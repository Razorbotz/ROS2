from setuptools import setup

package_name = 'stepper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
		['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='andrewburroughs17@gmail.com',
    description='Stepper package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'stepper_node = stepper.stepper_node:main',
        ],
    },
)
