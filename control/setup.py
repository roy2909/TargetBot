from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'config/tags.yaml',
                                   'launch/shoot_pins.launch.xml',
                                   'config/rviz.config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='495 group 2',
    maintainer_email='maximilianopalay2024@u.northwestern.edu',
    description='Control node for the final project of ME495',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = control.control:main'
        ],
    },
)
