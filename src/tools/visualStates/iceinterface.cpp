/*
 *  Copyright (C) 1997-2013 JDERobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 *  Authors : Borja Menéndez Moreno <b.menendez.moreno@gmail.com>
 *            José María Cañas Plaza <jmplaza@gsyc.es>
 *
 */

#include "iceinterface.h"

/*************************************************************
 * CONSTRUCTOR
 *************************************************************/
IceInterface::IceInterface ():serverType(ICE) {}

IceInterface::IceInterface ( ServerType serverType, std::string name, std::string proxyName, std::string rosTopic, std::string ip, std::string port, std::string interface ) {
	this->serverType = serverType;
	this->name = name;
	this->proxyName = proxyName;
	this->rosTopic = rosTopic;
	this->ip = ip;
	this->port = port;
	this->interface = interface;
}

/*************************************************************
 * DESTRUCTOR
 *************************************************************/
IceInterface::~IceInterface () {}

/*************************************************************
 * SETTERS
 *************************************************************/
void IceInterface::setAll ( ServerType serverType, std::string name, std::string proxyName, std::string rosTopic, std::string ip, std::string port, std::string interface ) {
	this->serverType = serverType;
	this->name = name;
	this->proxyName = proxyName;
	this->rosTopic = rosTopic;
	this->ip = ip;
	this->port = port;
	this->interface = interface;
}

void IceInterface::setServerType( ServerType serverType) {
	this->serverType = serverType;
}

void IceInterface::setName ( std::string name ) {
	this->name = name;
}

void IceInterface::setProxyName ( std::string name ){
	this->proxyName = name;
}

void IceInterface::setRosTopic(std::string topic) {
	this->rosTopic = topic;
}

void IceInterface::setIp ( std::string ip ) {
	this->ip = ip;
}

void IceInterface::setPort ( std::string port ) {
	this->port = port;
}

void IceInterface::setInterface ( std::string interface ) {
	this->interface = interface;
}

/*************************************************************
 * GETTERS
 *************************************************************/
ServerType IceInterface::getServerType() {
	return this->serverType;
}

std::string IceInterface::getName () {
	return this->name;
}

std::string IceInterface::getProxyName () {
	return this->proxyName;
}

std::string IceInterface::getRosTopic() {
	return this->rosTopic;
}

std::string IceInterface::getIp () {
	return this->ip;
}

std::string IceInterface::getPort () {
	return this->port;
}

std::string IceInterface::getInterface () {
	return this->interface;
}

/*************************************************************
 * ANOTHER FUNCTIONS
 *************************************************************/
bool IceInterface::equals ( IceInterface* iceinterface ) {
	return (this->name.compare(iceinterface->getName()) == 0);
}

bool IceInterface::equals ( std::string serverType, std::string name, std::string proxyName,
							std::string rosTopic, std::string ip, std::string port,
							std::string interface ) {
	ServerType tempServerType = ICE;
	if (serverType.compare("ICE") == 0)
		tempServerType = ICE;
	else if (serverType.compare("ROS") == 0)
		tempServerType = ROS;

	return ( tempServerType == this->serverType && (this->name.compare(name) == 0) && (this->proxyName.compare(proxyName) == 0)
			&& (this->rosTopic.compare(rosTopic) == 0) && (this->ip.compare(ip) == 0) && (this->port.compare(port) == 0)
			&& (this->interface.compare(interface) == 0) );
}