from setuptools import find_packages, setup

package_name = 'freenove_adc'

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
    maintainer='rene',
    maintainer_email='rene@digitale-dinge.de',
    description='publish ADC values of freenove Pi Car',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'adc_publish = freenove_adc.main:main',
        ],
    },
)
