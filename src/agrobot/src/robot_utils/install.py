import setuptools

setuptools.setup(
    name="robot_utils",
    version="0.1",
    scripts=['testing.py','services.py'],
    author="Caio Moreira",
    author_email="caioslppuo@gmail.com",
    description="Utils library used by Agrobot.",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent"
    ],
    install_requires=[
        'rosservice',
        'rosparam'
    ],
)