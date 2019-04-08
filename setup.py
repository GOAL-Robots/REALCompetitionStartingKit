from setuptools import setup
from setuptools.command.install import install as DistutilsInstall
from setuptools.command.egg_info import egg_info as EggInfo

import pybullet_data
import os
from distutils.dir_util import copy_tree


def cp_data():
    src_path = os.path.dirname(os.path.dirname(__file__) + "/realcomp/data/kuka_gripper_description" )
    dest_path = pybullet_data.getDataPath()
    copy_tree(src_path, dest_path)

class MyInstall(DistutilsInstall):
    def run(self):
        cp_data()
        DistutilsInstall.run(self)

class MyEgg(EggInfo):
    def run(self):
        cp_data()
        EggInfo.run(self)

setup(name='REALComp',
        version='0.1',
        cmdclass = {
            'install': MyInstall,
            'egg_info': MyEgg
            },
        install_requires=['gym', 'pybullet', 'numpy']
        )
