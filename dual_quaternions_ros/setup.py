from distutils.core import setup

def readme():
    with open('README.rst') as f:
        return f.read()

setup(name='dual_quaternions_ros',
      version='0.1.1',
      description='Dual quaternion ROS converter',
      long_description=readme(),
      url='http://github.com/Achllle/dual_quaternions_ros/tree/master/dual_quaternions_ros',
      author='Achille Verheye',
      author_email='achille.verheye@gmail.com',
      license='MIT',
      packages=['dual_quaternions_ros'],
      package_dir={'': 'src'})
