from setuptools import find_packages
from setuptools import setup

package_name = 'tm_get_status'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
#    packages=find_packages(exclude=['my_msgs']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='leowu',
    author_email='leo.wu@tm-robot.com',
    maintainer='Jufeng Wu',
    maintainer_email='leo.wu@tm-robot.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes which were previously in the ros2/examples repository '
        'but are now just used for demo purposes.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_talker = tm_get_status.image_pub:main',
            'status_talker = tm_get_status.get_status:main'
        ],
    },
)
