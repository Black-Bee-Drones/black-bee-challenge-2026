from setuptools import find_packages, setup

package_name = 'hang_the_hook'

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
    maintainer='arthur-xavier',
    maintainer_email='arthuraxribeiro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mangalarga = hang_the_hook.mangalarga:mangalarga',
            'hook_test = hang_the_hook.hook_test:hook_test',
            'plot_errors = hang_the_hook.utils.plot_errors:main',
        ],
    },
)
