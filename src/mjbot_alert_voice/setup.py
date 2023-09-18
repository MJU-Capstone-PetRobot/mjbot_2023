from setuptools import find_packages, setup

package_name = 'mjbot_alert_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [package_name + '.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drcl',
    maintainer_email='example@naver.com',  # Updated email address
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mjbot_alert_voice_comm = mjbot_alert_voice.main:main"
        ],
    },
)
