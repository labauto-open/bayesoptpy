from setuptools import setup, find_packages

def _requires_from_file(filename):
    return open(filename).read().splitlines()


setup(
    name='bayesopt_interfaces',
    version='0.1.0',
    author='Yuki Asano',
    author_email='yasano@g.ecc.u-tokyo.ac.jp',
    description='',
    packages=find_packages(),
    install_requires = _requires_from_file('requirements.txt')
)
