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
* scripts/ - various scrips to make development easier
* build/ - place where the makefile places the output
* settings/ - settings files

MAVLink Dependency
------------------

* PLEASE NOTE: YOU NEED TO DOWNLOAD THE UDENVERMAVLINK LIBRARY (https://github.com/josephlewis42/UDenverMavlink) IN ORDER TO COMPILE THIS APPLICATION *

To build on Linux, make sure you have the UDenverMavlink folder at the same level as the UDenverQGC

============================

Generate Documentation
----------------------

		$ cd src
		$ doxygen

the documentation will be placed in the doc directory.

============================

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

