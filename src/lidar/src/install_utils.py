import setuptools

setuptools.setup(
    name="lidar-utils", # Replace with your own username
    version="0.0.1",
    author="Lucas Garavaglia",
    author_email="lucasgrafimar@gmail.com",
    description="Library used by Agrobot.",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
