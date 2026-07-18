from setuptools import setup  # ,find_packages,

package_name = "basestation"

# packages=find_packages(exclude=["test"]),
setup(
    name=package_name,
    version="0.0.1",
    packages=["basestation"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "msgpack", "pyserial", "qtpy", "PySide6", "pyqtgraph"],
    zip_safe=True,
    maintainer="spacedays",
    maintainer_email="spacedays@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "basestation = basestation.basestation_gui:main",
        ],
    },
    tests_require=["pytest"],
)
