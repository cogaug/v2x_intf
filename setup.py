from setuptools import find_packages, setup

package_name = 'v2x_intf_pkg'

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
    maintainer='woojin',
    maintainer_email='woojin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v2x_intf_node = v2x_intf_pkg.V2XIntf:main',
            # 'v2x_subscriber = v2x_intf.v2x_subscriber:main',
            'v2x_msg_test = v2x_intf_pkg.V2XMsgTest:main',
        ],
    },
)
