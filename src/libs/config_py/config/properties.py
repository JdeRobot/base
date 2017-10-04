#
#  Copyright (C) 1997-2017 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#
__author__ = 'aitormf'

import yaml

class Properties:
	def __init__(self, cfg):
		self._config = cfg

	def getNode(self):
		return self._config

	def getProperty(self, name):

		names = name.split(".")

		return self._searchNode(self._config, names)

	def getPropertyWithDefault(self, name, dataDefault):

		try:
			return self.getProperty(name)
		
		except KeyError:
			return dataDefault


	def _searchNode(self, node, lst):
		name = lst.pop(0)

		nod = node[name]

		if (len(lst) > 0):
			return (self._searchNode(nod, lst))
		else:
			return nod

	def __str__(self):
		return yaml.dump(self._config)

