from setuptools import setup

setup(
    name='robotarm',
    version='0.0.1a',
    description='Python Module to plot and simulate an robot arm',
    url='',
    author='Robert Thomas',
    author_email='robtom_5@mac.com',
    license='MIT',
    packages=['robotarm'],
    classifiers=[
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.4',
    ],
    python_requires='>=3',
    install_requires=['numpy', 'matplotlib'],
    zip_safe=False
    )
