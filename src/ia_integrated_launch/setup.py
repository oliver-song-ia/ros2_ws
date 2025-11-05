from setuptools import find_packages, setup

package_name = 'ia_integrated_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加launch目录安装配置
        ('share/' + package_name + '/launch', ['launch/integrated_navigation.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ia',
    maintainer_email='ia@todo.todo',
    description='集成启动包 - 包含导航、SLAM、控制系统的统一启动文件',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
