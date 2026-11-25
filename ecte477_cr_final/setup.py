import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ecte477_cr_final'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='ECTE477 autonomous construction-site robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'destination_selector = ecte477_cr_final.destination_selector:main',
            'sheet_detector = ecte477_cr_final.sheet_detector:main',
            'zone_verifier = ecte477_cr_final.zone_verifier:main',
            'snapshot = ecte477_cr_final.snapshot:main',
            'hsv_tuner = ecte477_cr_final.hsv_tuner:main',
            'test_vision = ecte477_cr_final.test_vision:main',
            'record_zone = ecte477_cr_final.record_zone:main',
        ],
    },
)
