from setuptools import find_packages, setup
from pathlib import Path

# Leer los requisitos de runtime
req_file = Path(__file__).parent / "PythonRobotics" / "requirements" / "runtime.txt"
requirements = req_file.read_text().splitlines()

package_name = 'python_robotics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests*']),
    install_requires=['setuptools'] + requirements,
    zip_safe=True,
    maintainer='Iago Muñoz Varela',
    maintainer_email='imvarela17@esei.uvigo.es',
    description="""
    The Python Robotics package.
    It exposes several functionalities through a submodule.
    """,
    license='MIT',
)
