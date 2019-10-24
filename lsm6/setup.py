from setuptools import setup

package_name = 'lsm6'

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
    author='Conroy Cheers',
    author_email='cheers.conroy@gmail.com',
    maintainer='Conroy Cheers',
    maintainer_email='cheers.conroy@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='LSM6 I2C IMU driver',
    license='',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'lsm6_driver_node = lsm6.lsm6_driver_node:main',
        ],
    },
)
