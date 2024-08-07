from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dexhand_llm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),    
    ],
    install_requires=['setuptools','gtts','openai','pyaudio','SpeechRecognition','openai_session', 'sounddevice'],
    zip_safe=True,
    maintainer='Trent Shumay',
    maintainer_email='trent@iotdesignshop.com',
    description='A demonstration ROS2 package for controlling the DexHand with GPT4',
    license='CC BY-NC-SA 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_control = dexhand_llm_control.llm_control:main',
        ],
    },
)
