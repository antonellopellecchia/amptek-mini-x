from setuptools import setup

setup(
    name="amptek-mini-x",
    version="0.2",
    author="Friedemann Neuhaus",
    author_email="f@neuhaus-tech.de",
    py_modules=["minix"],
    install_requires=[
        "pyftdi",
    ],
)
