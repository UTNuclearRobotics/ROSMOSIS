from setuptools import find_packages, setup

package_name = 'pose_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='talal',
    maintainer_email='talal.alotaibi@utexas.edu',
    description='Viewpoint pose sampler services (helix, cone) for NBV mission planning',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'helix_service=pose_generator.helix_server:main',
            'cone_service=pose_generator.cone_server:main',
        ],
    },
)
