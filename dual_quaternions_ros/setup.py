from setuptools import setup, find_packages

def readme():
    with open('README.rst') as f:
        return f.read()

setup(name='dual_quaternions_ros',
      version='0.1.0',
      description='Dual quaternion ROS converter',
      long_description=readme(),
      url='http://github.com/Achllle/dual_quaternions_ros/tree/master/dual_quaternions_ros',
      author='Achille Verheye',
      author_email='achille.verheye@gmail.com',
      license='MIT',
      packages=find_packages(),
      install_requires=['dual_quaternions'],
      zip_safe=False,
      test_suite='nose.collector',
      tests_require='nose')
