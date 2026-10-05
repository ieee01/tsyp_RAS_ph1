from setuptools import find_packages, setup

package_name = "living_map_beacons"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/"+package_name]), ("share/"+package_name, ["package.xml"])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="LivingMap Team",
    maintainer_email="team@example.com",
    description="LivingMap Phase 1 package",
    license="Apache-2.0",
    entry_points={"console_scripts": {"beacon_manager = living_map_beacons.node:main", "beacon_visualizer = living_map_beacons.visualizer:main"}},
)
