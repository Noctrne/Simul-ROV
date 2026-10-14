from setuptools import find_packages, setup

package_name = 'simulrov'

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
    maintainer='dhipta',
    maintainer_email='dhiipt32@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'att_sim = simulrov.rov_att:main',
            'control_sim = simulrov.rov_control:main',
            'depth_sim = simulrov.rov_depth:main',
            'keyboard_sim = simulrov.rov_keyboard:main'
        ],
    },
)
