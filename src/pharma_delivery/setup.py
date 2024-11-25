from setuptools import find_packages, setup

package_name = 'pharma_delivery'

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
            'do_by_state=pharma_delivery.do_by_state:main',
            'state_update=pharma_delivery.state_update:main',
        ],
    },
)
