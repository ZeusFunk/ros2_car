from setuptools import setup
import os
from glob import glob
package_name = 'topic_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'topic_demo', 'launch'), glob('launch/*.launch.py')), #添加Launch文件支持
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesse',
    maintainer_email='jesse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = topic_demo.publisher:main',
            'subscriber = topic_demo.subscriber:main',
        ],
    },
)
