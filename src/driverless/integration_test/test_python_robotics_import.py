from importlib.util import find_spec

def test_python_robotics_import():
    spec = find_spec("PythonRobotics")
    assert spec is not None, "Could not import PythonRobotics. Make sure you run `colcon build` first."