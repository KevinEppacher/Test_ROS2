from setuptools import setup, find_packages

package_name = 'amr_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['modules', 'modules.*']),
    py_modules=[
        'scripts.publisher_node',
        'scripts.subscriber_node',
        'scripts.casadi_example',
        'scripts.optimal_control',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'optimal_control = scripts.optimal_control:main',
        ],
    },
)
