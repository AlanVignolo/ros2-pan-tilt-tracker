from setuptools import find_packages, setup

package_name = 'seguidor'

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
    maintainer='Alan Vignolo',
    maintainer_email='alanvignolo1999@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nodo_puente_serie = seguidor.nodo_puente_serie:main',
            'nodo_detector = seguidor.nodo_detector:main',
            'nodo_control = seguidor.nodo_control:main',        ],
    },
)
