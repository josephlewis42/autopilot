Readme
======

Purpose
-------

This is the source code for the University of Denver Helicopter autopilot, it
is meant to be used with QGroundControl and Mavlink to provide a highly 
customizable base for autopilot development in mid to high range helicopters, i.e.
non-civilian devices.

Environment
-----------

The autopilot was originally built to run on QNX, but extensive work has been put in to
get it to run on standard POSIX compliant systems, however these systems have not yet
been extensively tested in flight.


Software Information
====================

Directory Structure
-------------------


* src/ - Source files
* src/tests/ - test files for isolating components of the autopilot
* extern/ - External library headers
* lib/ - Precompiled external libraries

============================

To import the source into QNX Momentics IDE:

1. File->New->QNX C++ Project
2. Project name = Autopilot
3. Uncheck "Use default location"
4. Set location to src directory
5. Uncheck "Generate default file"
6. Click Next
7. Select appropriate Build Variant (e.g., x86 debug)
8. Click Finish

============================

Generate Documentation

$ cd src
$ doxygen

the documentation will be placed in the doc directory.

============================

MAVLink Dependency

The official MAVLink source can be found at https://github.com/mavlink/mavlink

However, this autopilot is built against a custom version which is not always in sync with the official version.
Therefore, it is recommended that to build the autopilot you clone our custom version available from
https://github.com/ancl/mavlink.  Note, we are currently using the v10release branch.

Similar to QGroundControl (http://qgroundcontrol.org) this software expects MAVLink to be stored in the same directory as the autopilot source.
In other words, after cloning the autopilot and mavlink you should see both of the folders in a single directory listing.


License
=======

Copyright (C) 2012 Bryan Godbolt
Copyright (C) 2013 Joseph Lewis III <joehms22@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

