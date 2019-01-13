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
    OPTION_CONTROL_ALL = True

    hebi_modules = [] # [(family, name), ...]

    # Discover modules
    lookup = hebi.Lookup()
    sleep(2) # Give the Lookup process 2 seconds to discover modules
    print('Modules found on network:')
    for entry in lookup.entrylist:
        hebi_modules.append((entry.family, entry.name))
        print('{0} | {1}'.format(entry.family, entry.name))

    if len(hebi_modules) <= 0:
        print('No module has been found...')
        exit()

    # Create a group from a set of names
    group = lookup.get_group_from_names([hebi_modules[0][0]], [hebi_modules[0][1]])
    if OPTION_CONTROL_ALL:
        group = lookup.get_group_from_names([m[0] for m in hebi_modules], [m[1] for m in hebi_modules])

    sleep(3)

    # Control
    pmax = 1.0 # rad
    pmin = -1.0 # rad
    nsteps = 200 # rad
    pstep = (pmax - pmin) / nsteps
    deltaT = 0.01 # sec
    group_command = hebi.GroupCommand(group.size)
    p = np.array([pmin + i * pstep for i in range(nsteps)])
    p_inv = p[[nsteps-1-i for i in range(nsteps)]]

    while True:
        for i in range(nsteps):
            pos = [p[i]]
            if OPTION_CONTROL_ALL:
                pos = [p[i] for m in hebi_modules]
            group_command.position = pos
            group.send_command(group_command)
            print('ctrl_pos = %.4f' % (p[i]));
            sleep(deltaT)
        for i in range(nsteps):
            pos = [p_inv[i]]
            if OPTION_CONTROL_ALL:
                pos = [p_inv[i] for m in hebi_modules]
            group_command.position = pos
            group.send_command(group_command)
            print('ctrl_pos = %.4f' % (p_inv[i]));
            sleep(deltaT)


if __name__ == '__main__':
    main()
