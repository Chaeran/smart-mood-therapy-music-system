from setuptools import find_packages, setup

package_name = 'moodlamp_sensors'

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
    maintainer='lana',
    maintainer_email='lana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'sensor_bridge_node = moodlamp_sensors.sensor_bridge_node:main',
		'mood_fusion_node = moodlamp_sensors.mood_fusion_node:main',
		'music_player_node = moodlamp_sensors.music_player_node:main',
		'ui_node = moodlamp_sensors.ui_node:main',
        ],
    },
)
