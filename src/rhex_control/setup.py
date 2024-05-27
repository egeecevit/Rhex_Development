from setuptools import find_packages, setup

package_name = 'rhex_control'

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
    maintainer='ege',
    maintainer_email='egeecevit@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["torque_control = rhex_control.torque_control:main",
                            "dynamic_joint_state_control = rhex_control.dynamic_joint_state_control:main",
        ],
    },
)
