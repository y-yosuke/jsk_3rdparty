#!/usr/bin/env python

import rospy
import os, sys, unittest, rostest

# https://stackoverflow.com/questions/10971033/backporting-python-3-openencoding-utf-8-to-python-2
if sys.version_info[0] > 2:
    # py3k
    pass
else:
    # py2
    import __builtin__
    def open(filename, encoding=None):
        return __builtin__.open(filename)

pkg_dir = os.path.abspath(os.path.join(os.path.realpath(__file__), os.pardir, os.pardir))
pkg_name = os.path.basename(pkg_dir)

class TestRospyNode(unittest.TestCase):

    def __init__(self, *args):
        unittest.TestCase.__init__(self, *args)

    def test_rosnode(self):
        __name__ = 'dummy'
        for scripts_dir in ['scripts', 'node_scripts']:
            full_scripts_dir = os.path.join(pkg_dir, scripts_dir)
            if not os.path.exists(full_scripts_dir):
                continue
            for filename in [f for f in map(lambda x: os.path.join(full_scripts_dir, x), os.listdir(full_scripts_dir)) if os.path.isfile(f) and f.endswith('.py')]:
                if filename.endswith('/helper.py'):
                    print("{} depends on dialogflow_task_executive. However, when we add this to the <depend> of package.xml, it appempts to build venv using 'dialogflow_task_executive/requirements.txt'. This requires having the same PYTHON_INTERPRETER for both dialogflow and chat ros package. The issue is that dialogflow_task_executive heavily relies on system Python modules, including ROS, making it difficult to use dialogflow with Python3 on Melodic".format(filename))
                    continue
                print("Check if {} is loadable".format(filename))
                # https://stackoverflow.com/questions/4484872/why-doesnt-exec-work-in-a-function-with-a-subfunction
                exec(open(filename, encoding='utf-8').read()) in globals(), locals()
                self.assertTrue(True)

if __name__ == '__main__':
    rostest.rosrun('test_rospy_node', pkg_name, TestRospyNode, sys.argv)
