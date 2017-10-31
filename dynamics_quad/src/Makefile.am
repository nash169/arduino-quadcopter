# Makefile.am
#
# This file is part of RoboBrain.
# Copyright (C) 2016 Bernardo Fichera
#
# RoboBrain is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RoboBrain is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with RoboBrain.  If not, see <http://www.gnu.org/licenses/>.

# Not sure about this. It probably needs to set the m4 folder for macros.
ACLOCAL_AMFLAGS = -I m4
# Nothing about it. It is likely connected to the previous command.
EXTRA_DIST = bootstrap
# Set executable output
bin_PROGRAMS = main

# Set executable sources
main_SOURCES = \
	main.cpp \
	robobee.cpp \
	controller.cpp \
	iomanager.cpp \
	plotter.cpp

# Set executable libraries linking
main_LDADD = \
	-ldynplot \
	-larmadillo \
  	$(BOOST_IOSTREAMS_LIB) \
  	$(BOOST_SYSTEM_LIB) \
  	$(BOOST_FILESYSTEM_LIB)