import importlib.util
from pathlib import Path


def generate_launch_description():
    sitl_launch = Path(__file__).with_name('sitl_vtol.launch.py')
    spec = importlib.util.spec_from_file_location('krac_mission_sitl_vtol_launch', sitl_launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
