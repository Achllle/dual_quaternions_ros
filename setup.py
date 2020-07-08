from setuptools import setup

def readme():
    with open('README.rst') as f:
        return f.read()

setup(name='dual_quaternions_ros',
      version='0.1.4',
      description='Dual quaternion ROS converter',
      long_description=readme(),
      url='http://github.com/Achllle/dual_quaternions_ros/tree/master/dual_quaternions_ros',
      author='Achille Verheye',
      author_email='achille.verheye@gmail.com',
      license='MIT',
      install_requires=['dual_quaternions>=0.3.1'],
      packages=['dual_quaternions_ros'],
      package_dir={'': 'src'},
      test_suite='nose.collector',
      tests_require='nose')
