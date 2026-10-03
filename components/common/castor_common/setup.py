from setuptools import find_packages, setup

package_name = "castor_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kiran Gunathilaka",
    maintainer_email="kiran.g@radskunkworks.com",
    description="robot.yaml loading and validation, castor-config, heartbeat node, launch helpers.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "castor-config = castor_common.cli:main",
            "heartbeat = castor_common.heartbeat:main",
        ],
    },
)
