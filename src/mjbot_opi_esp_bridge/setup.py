from setuptools import find_packages, setup

package_name = 'opi_esp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drcl',
    maintainer_email='mokhwasomssi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "opi_esp_comm = opi_esp.main:main",
            "fake_cmd = opi_esp.fake_cmd_pub:main"
        ],
    },
)
