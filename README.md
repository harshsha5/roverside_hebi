# Roverside HEBI Claw Manipulator Package

Roverside Assistance project HEBI claw manipulator package provides claw
manipulator control utilities for the rovers with HEBI actuators.


## Setup and Installation

Please refer to the below setup and installation process:

1. Clone this repository to your catkin workspace

2. Skim the HEBI document for basic understanding at
   http://docs.hebi.us/tools.html#Tools_APIs

3. Download the HEBI python tool from https://pypi.python.org/pypi/hebi-py

4. Extract the installation package and build it using the below commands

   ```
   tar -xf hebi-py-*.tar.gz
   cd Hebi-py-*
   python setup.py build
   ```

   You can also use the help command for more options

   ```
   python setup.py --help
   ```

5. Install the package

   ```
   sudo python setup.py install
   ```

6. Verify the installation by importing the HEBI module in python
   environment

   ```
   python
   >>> import hebi
   ```

   If no error pops up, then the python tool package installation is done

7. (Optional) To check if the HEBI hardware is actually working, first
   connect the HEBI actuators and your computer to the same network
   environment, and the go to the `test` directory in the cloned repository
   to run the test script

   ```
   roscd roverside_hebi/test
   python hebi_test.py
   ```

## Copyright

Below are the list of authors and copyright information.

* __Dicong Qiu__ (dq@cs.cmu.edu, http://www.davidqiu.com/)
* __Harsh Sharma__ (harshsha@andrew.cmu.edu)

Copyright (C) 2018, All rights reserved.
