from setuptools import setup

package_name = 'aidog_control'

setup(
    name=package_name,
    version='0.37.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Nola Donato',
    author_email='noladonato@gmail.com',
    maintainer='Nola Donato',
    maintainer_email='noladonato@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Actions and Services to control AI Dog',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'aidog_rotatezaxis_relative_server = aidog_control.aidog_rotatezaxis_relative_server:main',
            'aidog_rotatezaxis_relative_client = aidog_control.aidog_rotatezaxis_relative_client:main',
            'aidog_get_camera_image = aidog_control.aidog_camera_image_service:main',
            'aidog_camera_image_client = aidog_control.aidog_camera_image_client:main',
            'aidog_web_camera_image = aidog_control.aidog_web_camera_image_service:main',
        ],
    },
)