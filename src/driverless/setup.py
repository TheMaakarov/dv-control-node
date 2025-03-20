from setuptools import find_packages, setup

package_name = 'driverless'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Iago Muñoz Varela',
    maintainer_email='imvarela17@esei.uvigo.es',
    description="""
    The Driverless package.
    It contains only the Control Node,
    which determines the actions to be performed
    over a provided Spline and publishes these actions
    so that the Action Node can consume them.
    """,
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control.control_node:main'
        ],
    },
)
