from setuptools import find_packages, setup

package_name = 'cbf_safety'

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
    maintainer='Adithya Pothula',
    maintainer_email='adithyapothula123@gmail.com',
    description='CBF_Safety',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'safety_node = cbf_safety.cbf_safety_node:main',
        'safety_filter = cbf_safety.safety_filter:main',
	    'simple_odom = cbf_safety.simple_odom:main',
        'homing = cbf_safety.homing:main',
        'choreographer_pause = cbf_safety.choreographer_pause:main',
        'cbf_tester = cbf_safety.cbf_tester:main',
        'plt_cbf = cbf_safety.plt_cbf:main',
        'plt_three_way = cbf_safety.plt_three_way:main',
        ],
    },
)
