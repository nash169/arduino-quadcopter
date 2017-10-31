# configure.ac
#
# This file is part of quad_controller.
# Copyright (C) 2016 Bernardo Fichera
#
# quad_controller is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# quad_controller is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with quad_controller.  If not, see <http://www.gnu.org/licenses/>.

# Require a minimum version of Autotools installed on the machine
AC_PREREQ([2.59])
# Project's name and release
AC_INIT([quad_controller], [1.0.0])
# ? Not sure about this macro.. might be deleted
AC_CONFIG_HEADERS([config.h])
# Define source dir
AC_CONFIG_SRCDIR([src/main.cpp])
# Run macros for makefile
AM_INIT_AUTOMAKE
# Conditional passed to the makefile
AM_CONDITIONAL([DARWIN], [test $(uname) = Darwin])
# Do compilation test in the specifed language
AC_LANG([C++])
# Determine C++ compiler to use
AC_PROG_CXX([g++-7 g++-6 g++])
# Test whether the library is available. It does not work so far.
AC_CHECK_LIB([armadillo], [_init])
AC_CHECK_LIB([dynplot], [_init])
# Check if the header is compilable
AC_CHECK_HEADER([armadillo],, [AC_MSG_ERROR([armadillo header not found in your system.])])
AC_CHECK_HEADER([dynplot.hh],, [AC_MSG_ERROR([dynplot.hh header not found in your system.])])
# Set folder for imported macros
AC_CONFIG_MACRO_DIR([m4])
# Check Boost C++ libraries. Automatically passes BOOST_$LIB_NAME$_LIB variable to makefile
AX_BOOST_BASE([1.53],,[AC_MSG_ERROR([projectX needs Boost, but it was not found in your system])])
AX_BOOST_IOSTREAMS([1.53],, [AC_MSG_ERROR([gnuplot-iostream needs boost_iostreams but it was not found on your system.])])
AX_BOOST_SYSTEM([1.53],, [AC_MSG_ERROR([gnuplot-iostream needs boost_system but it was not found on your system.])])
AX_BOOST_FILESYSTEM([1.53],, [AC_MSG_ERROR([gnuplot-iostream needs boost_filesystem but it was not found on your system.])])
# Set makefiles location
AC_CONFIG_FILES([Makefile src/Makefile])
# To call at the end of configure.ac
AC_OUTPUT
