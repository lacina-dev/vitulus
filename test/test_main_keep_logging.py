#!/usr/bin/env python3
"""nodes/main keeps its own logging when it starts launch files (T36).

roslaunch.configure_logging() replaces the handlers of the loggers `root` and
`rosout`; main calls it for every launch file it starts, after init_node, so
its own warnings never reached /rosout. The node wraps the call and puts its
handlers back (the same wrapper as navi_transform, navi_man, dock_manager).
Only the module is loaded here: nothing is launched, no master is contacted.

Run: python3 test/test_main_keep_logging.py
"""
import importlib.machinery
import importlib.util
import logging
import os
import sys
import tempfile

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'
os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='t36_main_log_')
HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, '..', 'nodes', 'main')

import roslaunch  # noqa: E402

ORIGINAL = roslaunch.configure_logging
loader = importlib.machinery.SourceFileLoader('vitulus_main_node', MAIN)
spec = importlib.util.spec_from_loader('vitulus_main_node', loader)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class Catch(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.msgs = []

    def emit(self, record):
        self.msgs.append(record.getMessage())


def main():
    fails = []

    def check(name, cond):
        print('  %-66s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    w = roslaunch.configure_logging
    check('the wrapper is installed at import, around the real call',
          getattr(w, '_keeps_node_logging', False) and w._original is ORIGINAL)
    root, rosout = logging.getLogger(), logging.getLogger('rosout')
    for lg in (root, rosout):
        for h in list(lg.handlers):
            lg.removeHandler(h)
    a, b = Catch(), Catch()
    root.addHandler(a)
    rosout.addHandler(b)
    w('uuid-1')
    w('uuid-2')
    logging.getLogger('rosout').warning('a warning of main')
    check('after two launches the handlers of root and rosout are the node\'s own',
          root.handlers == [a] and rosout.handlers == [b])
    check('a warning reaches both', a.msgs == ['a warning of main'] and b.msgs == ['a warning of main'])
    src = open(MAIN).read()
    check('both call sites go through roslaunch.configure_logging',
          src.count('roslaunch.configure_logging(self.uuid)') == 1
          and src.count('roslaunch.configure_logging(uuid)') == 1)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
