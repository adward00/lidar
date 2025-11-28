from setuptools import find_packages, setup

package_name = 'datapack'

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
    maintainer='mg0220',
    maintainer_email='mg0220@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'data_node = datapack.data_node:main',
            'subscriber = datapack.subscriber:main',
            'publisher = datapack.publisher:main',
            'ros = datapack.ros:main',
        ],
    },
)
