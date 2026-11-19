"""ROS 2 ament installation; project metadata stays in pyproject.toml."""
from glob import glob
from setuptools import find_packages, setup
from setuptools.dist import Distribution


class RosCompatibleDistribution(Distribution):
    """Keep Python requirements literal-safe for colcon's setup inspection.

    setuptools 68 reads PEP 621 requires-python as a SpecifierSet, whereas
    Jazzy's colcon_python_setup_py serializes Distribution.__dict__ using repr
    then ast.literal_eval. Normalize this one metadata field to its equivalent
    string after parsing; wheel Requires-Python and pip version checks remain.
    """
    def parse_config_files(self, *args, **kwargs):
        result = super().parse_config_files(*args, **kwargs)
        for holder in (self, self.metadata):
            requirement = getattr(holder, 'python_requires', None)
            if requirement is not None:
                holder.python_requires = str(requirement)
        return result

package_name = "tg3_pico_dualarm"

setup(
    distclass=RosCompatibleDistribution,
    name=package_name,
    version="0.4.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name, ["README.md", "NOTICE", "LICENSE"]),
    ],
    zip_safe=False,
)
