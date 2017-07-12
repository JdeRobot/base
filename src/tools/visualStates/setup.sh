#!/bin/sh
#
# Copyright (c) 2016
# Author: Samuel Rey <samuel.rey.escudero@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>

#C++ Dependencies
sudo mkdir /usr/local/include/jderobot/visualStates
sudo cp *.h  /usr/local/include/jderobot/visualStates
sudo mkdir /usr/local/include/jderobot/visualStates/popups
sudo cp popups/*.h /usr/local/include/jderobot/visualStates/popups/
sudo mkdir /usr/local/share/jderobot/glade/visualStates
sudo cp gui/* /usr/local/share/jderobot/glade/visualStates/
sudo cp getinterfaces.sh /usr/local/bin

#Python Dependencies
sudo mkdir /usr/local/share/jderobot/python/visualStates_py
sudo cp *.py /usr/local/share/jderobot/python/visualStates_py
sudo mkdir /usr/local/share/jderobot/python/visualStates_py/gui
sudo cp gui/*.py /usr/local/share/jderobot/python/visualStates_py/gui

sudo cp visualStates /usr/local/bin

echo "Dependencies installed"