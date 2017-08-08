#!/bin/sh
#
# Copyright (c) 2016
# Author: Samuel Rey <samuel.rey.escudero@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>
INSTALL_PREFIX=/opt/jderobot
#C++ Dependencies
sudo mkdir $INSTALL_PREFIX/include/jderobot/visualStates
sudo cp *.h  $INSTALL_PREFIX/include/jderobot/visualStates
sudo mkdir $INSTALL_PREFIX/include/jderobot/visualStates/popups
sudo cp popups/*.h $INSTALL_PREFIX/include/jderobot/visualStates/popups/
sudo mkdir $INSTALL_PREFIX/share/jderobot/glade/visualStates
sudo cp gui/* $INSTALL_PREFIX/share/jderobot/glade/visualStates/
sudo cp getinterfaces.sh $INSTALL_PREFIX/bin

#Python Dependencies
sudo mkdir $INSTALL_PREFIX/share/jderobot/python/visualStates_py
sudo cp *.py $INSTALL_PREFIX/share/jderobot/python/visualStates_py
sudo mkdir $INSTALL_PREFIX/share/jderobot/python/visualStates_py/gui
sudo cp gui/*.py $INSTALL_PREFIX/share/jderobot/python/visualStates_py/gui

sudo cp visualStates $INSTALL_PREFIX/bin

echo "Dependencies installed"
