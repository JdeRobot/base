#!/bin/sh
#
# Copyright (c) 2016
# Author: Samuel Rey <samuel.rey.escudero@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>

#C++ Dependencies
sudo mkdir /usr/local/include/jderobot/visualHFSM
sudo cp *.h  /usr/local/include/jderobot/visualHFSM/
sudo mkdir /usr/local/include/jderobot/visualHFSM/popups
sudo cp popups/*.h /usr/local/include/jderobot/visualHFSM/popups/
sudo mkdir /usr/local/share/jderobot/glade/visualHFSM
sudo cp gui/* /usr/local/share/jderobot/glade/visualHFSM/
sudo cp getinterfaces_new.sh /usr/local/bin

#Python Dependencies
sudo mkdir /usr/local/share/jderobot/python/visualHFSM_py
sudo cp *.py /usr/local/share/jderobot/python/visualHFSM_py
sudo mkdir /usr/local/share/jderobot/python/visualHFSM_py/gui
sudo cp gui/*.py /usr/local/share/jderobot/python/visualHFSM_py/gui

sudo cp visualHFSM /usr/local/bin

echo "Dependencies installed"
