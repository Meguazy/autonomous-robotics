from setuptools import find_packages, setup

package_name = 'dobot_cv'

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
    maintainer='finucci',
    maintainer_email='finucci.francesco98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = dobot_cv.camera_feed_publisher:main',
            'camera_processor = dobot_cv.camera_feed_processor:main',
            'pick_and_place_sub = dobot_cv.pick_and_place:main',
            'semaphore_publisher = dobot_cv.semaphore_publisher:main',
        ],
    },
)
