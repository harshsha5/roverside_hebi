#!/usr/bin/env python

"""
hebi_test.py
Test the basic control for HEBI actuator.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2018, the Moon Wreckers. All rights reserved."


import hebi
from time import sleep
import numpy as np

import IPython


def main():
    hebi_modules = [('arm', 'joint_0'), ('arm', 'joint_1')]

    # Discover modules
    count_found = 0
    lookup = hebi.Lookup()
    sleep(2) # Give the Lookup process 2 seconds to discover modules
    print('Modules found on network:')
    for entry in lookup.entrylist:
        print('{0} | {1}'.format(entry.family, entry.name))
        if (entry.family, entry.name) in hebi_modules:
            count_found = count_found + 1

    if len(hebi_modules) != count_found:
        print('Some HEBI module(s) is missing (%d/%d)...' % (count_found, len(hebi_modules)))
        exit()

    # Create a group from a set of names
    group = lookup.get_group_from_names([m[0] for m in hebi_modules], [m[1] for m in hebi_modules])
    print('Control group created...')
    print(group)

    sleep(3)

    exit()


if __name__ == '__main__':
    main()
