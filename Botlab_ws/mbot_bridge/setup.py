# Third Party
import os
import subprocess
import setuptools
from setuptools.command.build_ext import build_ext


class CMakeBuild(build_ext):
    def run(self):
        # Make sure that CMake is installed
        if not os.path.exists('build'):
            os.makedirs('build')
        subprocess.check_call(['cmake', '../mbot_cpp'], cwd='build')
        subprocess.check_call(['make'], cwd='build')


class CMakeInstall(build_ext):
    def run(self):
        # Make sure that CMake is installed
        if not os.path.exists('build'):
            raise Exception("Library must be built first. Run: python setup.py build_ext")
        subprocess.check_call(['sudo', 'make', 'install'], cwd='build')


setuptools.setup(
    cmdclass={'build_ext': CMakeBuild,
              'install_ext': CMakeInstall},
)
