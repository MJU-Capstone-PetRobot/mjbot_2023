from setuptools import setup

package_name = 'mjbot_random_move'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='The mjbot_random_move package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_move = mjbot_random_move.random_move:main'
        ],
    },
)
