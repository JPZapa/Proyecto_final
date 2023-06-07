from setuptools import setup

package_name = 'proyecto_master_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snowartz',
    maintainer_email='snowartz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['master = proyecto_master_pkg.master:main',
        'Manipulator_service = proyecto_master_pkg.Manipulator_service:main',
        'Navigation_service = proyecto_master_pkg.Navigation_service:main',
        'Perception_service = proyecto_master_pkg.Perception_service:main',
        'test_services = proyecto_master_pkg.test_services:main',
        ],
    },
)
