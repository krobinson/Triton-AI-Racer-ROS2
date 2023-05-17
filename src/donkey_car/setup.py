from setuptools import setup
import os
from glob import glob

package_name = 'donkey_car'
sub_modules = 'external/donkeycar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='kenrobinsonter@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'donkey_car_node = donkey_car.donkey_car_node:main',
            'donkey_car_calibrate_node = donkey_car.donkey_car_calibrate_node:main',
            'donkey_car_cnnactivation_node = donkey_car.donkey_car_cnnactivation_node:main',
            'donkey_car_createcar_node = donkey_car.donkey_car_createcar_node:main',
            'donkey_car_createjs_node = donkey_car.donkey_car_createjs_node:main',
            'donkey_car_findcar_node = donkey_car.donkey_car_findcar_node:main',
            'donkey_car_makemovie_node = donkey_car.donkey_car_makemovie_node:main',
            'donkey_car_models_node = donkey_car.donkey_car_models_node:main',
            'donkey_car_train_node = donkey_car.donkey_car_train_node:main',
            'donkey_car_tubclean_node = donkey_car.donkey_car_tubclean_node:main',
            'donkey_car_tubhist_node = donkey_car.donkey_car_tubhist_node:main',
            'donkey_car_tubplo_node = donkey_car.donkey_car_tubplo_node:main',
            'donkey_car_ui_node = donkey_car.donkey_car_ui_node:main',
            'donkey_car_update_node = donkey_car.donkey_car_update_node:main'

        ],
    },
)
