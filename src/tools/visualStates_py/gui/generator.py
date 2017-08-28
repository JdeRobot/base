'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
class Generator():

    def __init__(self):
        pass

    def generateCfg(self, cfgStr):
        cfgStr.append('# 0 -> Deactivate, 1 -> Ice , 2 -> ROS\n')
        for cfg in self.configs:
            proxyName = None
            if 'proxyName' not in cfg:
                proxyName = cfg['interface']
            else:
                proxyName = cfg['proxyName']
            cfgStr.append('automata.')
            cfgStr.append(cfg['name'])
            cfgStr.append('.Server=')
            if cfg['serverType'] == 'ice':
                cfgStr.append('1')
            elif cfg['serverType'] == 'ros':
                cfgStr.append('2')
            cfgStr.append('\n')

            cfgStr.append('automata.')
            cfgStr.append(cfg['name'])
            cfgStr.append('.Proxy=')
            cfgStr.append(proxyName)
            cfgStr.append(':default -h ')
            cfgStr.append(cfg['ip'])
            cfgStr.append(' -p ')
            cfgStr.append(str(cfg['port']))
            cfgStr.append('\n')

            if 'topic' not in cfg:
                cfg['topic'] = ''
            cfgStr.append('automata.')
            cfgStr.append(cfg['name'])
            cfgStr.append('.Topic=')
            cfgStr.append(cfg['topic'])
            cfgStr.append('\n')

            cfgStr.append('automata.')
            cfgStr.append(cfg['name'])
            cfgStr.append('.Name=')
            cfgStr.append(cfg['name'])
            cfgStr.append('\n')

        return cfgStr


    def generateUserFunctions(self, functionsStr):
        for state in self.states:
            functionsStr.append(state.getFunctions())
            functionsStr.append('\n')
        return functionsStr

    def sanitizeVar(self, var):
        return var.replace(' ', '_')