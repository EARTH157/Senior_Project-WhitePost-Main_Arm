import os
from glob import glob
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
        
        # 🚩 1. ติดตั้งไฟล์ .pt (ใช้แบบเดิมได้)
        (os.path.join('share', package_name), glob('processor_unit/*.pt')),
        
        # 🚩 2. ติดตั้งไฟล์ในโฟลเดอร์ NCNN (แก้ให้เจาะจงเพื่อเลี่ยง __pycache__)
        (os.path.join('share', package_name, 'elevator_btn_4_best_ncnn_model'), 
         glob('processor_unit/elevator_btn_4_best_ncnn_model/*.bin') + 
         glob('processor_unit/elevator_btn_4_best_ncnn_model/*.param') + 
         glob('processor_unit/elevator_btn_4_best_ncnn_model/*.yaml') + 
         glob('processor_unit/elevator_btn_4_best_ncnn_model/*.py')), # เผื่อมีไฟล์ python ในนั้น

        # 3. ติดตั้งไฟล์ launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspi-earth',
    maintainer_email='raspi-earth@todo.todo',
    description='Senior Project: WhitePost - Processor Unit',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_processor = processor_unit.main_processor:main',
            'uart_zero_two_w = processor_unit.uart_zero_two_w:main',
            'steam_cam = processor_unit.steam_opencv:main',
            'uart_command_and_status_to_pi5 = processor_unit.uart_command_and_status_to_pi5:main',
            'esp32_bridge_node = processor_unit.esp32_bridge_node:main',
            'camera_front = processor_unit.camera_front:main',
            'camera_back = processor_unit.camera_back:main',
            'ws_viewer_front = processor_unit.ws_viewer_front:main',
            'ws_viewer_back = processor_unit.ws_viewer_back:main',
            'test = processor_unit.test:main',
            'force_receiver_node = processor_unit.force_receiver_node:main',
        ],
    },
)