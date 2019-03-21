from setuptools import setup
from setuptools.command.build_ext import build_ext as _build_ext


def readme():
    with open('README.rst') as f:
        return f.read()


# hack for https://github.com/moble/quaternion/issues/99
class build_ext(_build_ext):
    def finalize_options(self):
        _build_ext.finalize_options(self)
        # Prevent numpy from thinking it is still in its setup process:
        __builtins__.__NUMPY_SETUP__ = False
        import numpy
        self.include_dirs.append(numpy.get_include())


setup(name='dual_quaternions_ros',
      version='0.1',
      description='Dual quaternion implementation for use with ROS',
      long_description=readme(),
      url='http://github.com/Achllle/dual_quaternions_ros',
      author='Achille Verheye',
      author_email='achille.verheye@gmail.com',
      license='MIT',
      packages=['dual_quaternions_ros'],
      cmdclass={'build_ext': build_ext},
      setup_requires=['numpy'],
      install_requires=['numpy', 'numpy-quaternion', 'numba'],
      zip_safe=False,
      test_suite='nose.collector',
      tests_require='nose')
