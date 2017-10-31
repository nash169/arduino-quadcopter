#!/bin/sh

# Add libtool support to package
if [ `uname -s` = Darwin ] ; then
  LIBTOOLIZE=glibtoolize # for macOS
else
  LIBTOOLIZE=libtoolize # for GNU/Linux distros
fi

# Create AUTHORS, NEWS, README, ChangeLog files
echo adding AUTHORS NEWS README ChangeLog files when not existing &&
touch AUTHORS NEWS README ChangeLog &&
# Create aclocal.m4 file and autom4te.cache
echo aclocal &&
aclocal &&
# Create ltmain.sh file and other files in m4 folder
echo $LIBTOOLIZE --copy --automake &&
$LIBTOOLIZE --copy --automake &&
# Create config.h.in file
echo autoheader &&
autoheader &&
# Create configure file
echo autoconf &&
autoconf &&
# Create Makefile.in, src/Makefile.in, COPYING, INSTALL, config.guess, config.sub, ar-lib, compile, depcomp, install-sh, missing files
echo automake --copy --add-missing &&
automake --copy --add-missing &&
echo Now run build shell script. It will automatically configure and compile the program.