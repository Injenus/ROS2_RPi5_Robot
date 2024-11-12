from setuptools import find_packages, setup
import os
import sys

venv_path = os.path.join(os.path.dirname(__file__), '.venv', 'lib', 'python3.11', 'site-packages')
sys.path.insert(0, venv_path)
package_name = 'image_processing'

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
    maintainer='inj',
    maintainer_email='injenus777@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_frames=image_processing.get_frames:main',
            'view_raws=image_processing.view_raws:main',
            'save_videos=image_processing.save_videos:main',
            'arucos=image_processing.arucos:main',
            'line_segments=image_processing.line_segments:main',
            'signs=image_processing.signs:main',
            'arrows=image_processing.arrows:main'
        ],
    },
)
