from setuptools import find_packages, setup

package_name = 'processor_unit'

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
    maintainer='raspi-earth',
    maintainer_email='raspi-earth@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main_processor = processor_unit.main_processor:main',
            'test = processor_unit.test:main',
            'test1 = processor_unit.test1:main',
            'uart_zero_two_w = processor_unit.uart_zero_two_w:main',
        ],
    },
)
