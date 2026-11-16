from setuptools import setup

package_name = 'robocar_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Chemin',
    maintainer_email='jason.chemin@epitech.eu',
    description='Suivi de flèches au ruban blanc et arrêt de sécurité LiDAR.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arrow_follow_node = robocar_autonomy.arrow_follow_node:main',
            'lidar_safety_gate = robocar_autonomy.lidar_safety_gate:main',
        ],
    },
)
