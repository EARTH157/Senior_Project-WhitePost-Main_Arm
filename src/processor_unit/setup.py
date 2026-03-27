import os
from glob import glob # อย่าลืม import glob เพิ่มที่ด้านบน
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
        
        # 🚩 เพิ่มบรรทัดนี้ เพื่อติดตั้งไฟล์ .pt ไว้ในโฟลเดอร์ share ของแพ็กเกจ
        (os.path.join('share', package_name), glob('processor_unit/*.pt')),
        
        # ถ้ามีไฟล์ launch ก็ใส่ไปด้วย (ถ้ายังไม่มีก็ข้ามไปครับ)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'uart_zero_two_w = processor_unit.uart_zero_two_w:main',
            'steam_cam = processor_unit.steam_opencv:main',
            'uart_command_and_status_to_pi5 = processor_unit.uart_command_and_status_to_pi5:main',
            'esp32_bridge_node = processor_unit.esp32_bridge_node:main',
            'camera_front = processor_unit.camera_front:main',
            'camera_back = processor_unit.camera_back:main',
            'test = processor_unit.test:main',
            'force_receiver_node = processor_unit.force_receiver_node:main',
        ],
    },
)
