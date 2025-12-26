from setuptools import setup

package_name = 'tm_mod_urdf'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ken Tsai',
    maintainer_email='ken.tsai@tm-robot.com',
    description='tm_mod_urdf',
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'modify_urdf = tm_mod_urdf.modify_urdf:main',
            'modify_xacro = tm_mod_urdf.modify_xacro:main'
        ],
    },
)
