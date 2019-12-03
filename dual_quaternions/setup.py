from setuptools import setup, find_packages

def readme():
    with open('README.rst') as f:
        return f.read()

setup(name='dual_quaternions',
      version='0.2.4',
      description='Dual quaternion implementation',
      long_description=readme(),
      url='http://github.com/Achllle/dual_quaternions_ros/tree/master/dual_quaternions',
      author='Achille Verheye',
      author_email='achille.verheye@gmail.com',
      license='MIT',
      packages=find_packages(exclude=('tests')),
      install_requires=['numpy', 'scipy', 'numpy-quaternion', 'numba'],
      zip_safe=False,
      test_suite='nose.collector',
      tests_require='nose')
