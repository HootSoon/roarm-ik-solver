from setuptools import setup, find_packages

setup(
    name='roarm_ik',
    version='0.1.0',
    description='A 3D Inverse Kinematics library for the Waveshare RoArm-M3',
    author='HootSoon: Hudson Reeves',
    packages=find_packages(),
    install_requires=[
        'roarm_sdk',
    ],
)