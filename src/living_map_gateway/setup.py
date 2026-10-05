from setuptools import find_packages, setup

package_name = "living_map_gateway"
setup(
    name=package_name,
    version="0.1.1",
    packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/"+package_name]), ("share/"+package_name, ["package.xml"])],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer="LivingMap Team",
    maintainer_email="team@example.com",
    description="LivingMap Phase 1 package",
    license="Apache-2.0",
    entry_points={"console_scripts": {"gateway = living_map_gateway.node:main"}},
)
