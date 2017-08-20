from gui.transitiontype import TransitionType
from gui.generator import Generator
import os

class CppGenerator(Generator):
    def __init__(self, libraries, configs, interfaceHeaders, states):
        self.libraries = libraries
        self.configs = configs
        self.interfaceHeaders = interfaceHeaders
        self.states = states

    def generate(self, projectPath, projectName):
        stringList = []
        self.generateHeaders(stringList)
        self.generateEnums(stringList)
        self.generateVariables(stringList)
        self.generateShutdownMethod(stringList)
        self.generateUserFunctions(stringList)
        self.generateRunTimeGui(stringList)
        self.generateStates(stringList)
        self.generateAutomataGui(stringList)
        self.generateReadArgs(stringList)
        self.generateMain(stringList)
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '.cpp', 'w')
        fp.write(sourceCode)
        fp.close()

        stringList = []
        self.generateCfg(stringList)
        cfgString = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '.cfg', 'w')
        fp.write(cfgString)
        fp.close()

        stringList = []
        self.generateCmake(stringList, projectName)
        cmakeString = ''.join(stringList)
        fp = open(projectPath + os.sep + 'CMakeLists.txt', 'w')
        fp.write(cmakeString)
        fp.close()


    def generateHeaders(self, headers):
        headers.append('#include <Ice/Ice.h>\n')
        headers.append('#include <IceUtil/IceUtil.h>\n')
        headers.append('#include <easyiceconfig/EasyIce.h>\n')
        headers.append('#include <jderobot/visualStates/automatagui.h>\n')
        for lib in self.libraries:
            headers.append('#include <')
            headers.append(lib.strip('\n'))
            headers.append('>\n')

        # generate interface headers
        for cfg in self.configs:
            headers.append('#include <jderobot/')
            headers.append(self.interfaceHeaders[cfg['proxyName']].strip('\n'))
            headers.append('.h>\n')

        headers.append('\n')

        return headers

    def generateEnums(self, enums):
        for state in self.states:
            # create enumerations for states
            enums.append('typedef enum State_')
            enums.append(str(state.id))
            enums.append(' {\n')
            for childState in state.getChildren():
                enums.append('\t')
                enums.append(self.sanitizeVar(childState.name))
                enums.append(',\n')
            enums.append('} State_')
            enums.append(str(state.id))
            enums.append(';\n\n')

            # create names arrays
            enums.append('const char* State_Names_')
            enums.append(str(state.id))
            enums.append('[] = {\n')
            for childState in state.getChildren():
                enums.append('\t"')
                enums.append(self.sanitizeVar(childState.name))
                enums.append('",\n')
            enums.append('};\n\n')

        return enums

    def generateVariables(self, vars):
        for state in self.states:
            vars.append('pthread_t thr_')
            vars.append(str(state.id))
            vars.append(';\n')

        # create gui thread and variable
        vars.append('pthread_t thr_gui;\n\n')
        vars.append('AutomataGui* automatagui;\n')
        vars.append('bool displayGui = false;\n\n')

        for state in self.states:
            vars.append('bool run')
            vars.append(str(state.id))
            vars.append(' = true;\n')
        vars.append('\n')

        for state in self.states:
            initialChild = state.getInitialChild()
            vars.append('State_')
            vars.append(str(state.id))
            vars.append(' sub_')
            vars.append(str(state.id))
            vars.append(' = ')
            vars.append(self.sanitizeVar(initialChild.name))
            vars.append(';\n')
        vars.append('\n')

        for cfg in self.configs:
            vars.append('jderobot::')
            vars.append(cfg['interface'])
            vars.append('Prx ')
            vars.append(cfg['name'])
            vars.append(';\n')
        vars.append('\n')

        return vars

    def generateShutdownMethod(self, methods):
        methods.append('void shutDown() {\n')
        for state in self.states:
            methods.append('\trun')
            methods.append(str(state.id))
            methods.append(' = false;\n')
        methods.append('\tautomatagui->close();\n')
        methods.append('}\n\n')
        return methods


    def generateRunTimeGui(self, runtimegui):
        runtimegui.append('std::list<GuiSubautomata> createGuiSubAutomataList(){\n')
        runtimegui.append('\tstd::list<GuiSubautomata> guiSubautomataList;\n\n')

        stateVar = ''
        for state in self.states:
            runtimegui.append('\tGuiSubautomata* ')
            runtimegui.append('guiSubautomata')
            runtimegui.append(str(state.id))
            stateVar = 'guiSubautomata'+str(state.id)
            runtimegui.append(' = new GuiSubautomata(')
            runtimegui.append(str(state.id))
            runtimegui.append(', ')
            if state.parent is not None:
                runtimegui.append(str(state.parent.id))
            else:
                runtimegui.append('0')
            runtimegui.append(');\n\n')

            for childState in state.getChildren():
                runtimegui.append('\t')
                runtimegui.append(stateVar)
                runtimegui.append('->newGuiNode(')
                runtimegui.append(str(childState.id))
                runtimegui.append(', ')
                runtimegui.append(str(state.id)) #todo: check this, sonAutomataId
                runtimegui.append(', ')
                runtimegui.append(str(childState.x))
                runtimegui.append(', ')
                runtimegui.append(str(childState.y))
                runtimegui.append(');\n\t')
                runtimegui.append(stateVar)
                runtimegui.append('->setIsInitialLastGuiNode(')
                if childState.initial:
                    runtimegui.append('true')
                else:
                    runtimegui.append('false')
                runtimegui.append(');\n\t')
                runtimegui.append(stateVar)
                runtimegui.append('->setNameLastGuiNode("')
                runtimegui.append(childState.name)
                runtimegui.append('");\n\n')

            # now iterate over the transitions
            for childState in state.getChildren():
                for tran in childState.getOriginTransitions():
                    runtimegui.append('\tPoint* ')
                    originVar = 'origin'+str(childState.id)+str(tran.id)
                    runtimegui.append(originVar)
                    runtimegui.append(' = new Point(')
                    runtimegui.append(str(tran.origin.x))
                    runtimegui.append(', ')
                    runtimegui.append(str(tran.origin.y))
                    runtimegui.append(');\n')

                    runtimegui.append('\tPoint* ')
                    destVar = 'dest'+str(childState.id)+str(tran.id)
                    runtimegui.append(destVar)
                    runtimegui.append(' = new Point(')
                    runtimegui.append(str(tran.destination.x))
                    runtimegui.append(', ')
                    runtimegui.append(str(tran.destination.y))
                    runtimegui.append(');\n')

                    runtimegui.append('\tPoint* ')
                    midVar = 'midPoint'+str(childState.id)+str(tran.id)
                    runtimegui.append(midVar)
                    runtimegui.append(' = new Point(')
                    runtimegui.append(str(tran.x))
                    runtimegui.append(', ')
                    runtimegui.append(str(tran.y))
                    runtimegui.append(');\n')

                    runtimegui.append('\t')
                    runtimegui.append(stateVar)
                    runtimegui.append('->newGuiTransition(*')
                    runtimegui.append(originVar)
                    runtimegui.append(', *')
                    runtimegui.append(destVar)
                    runtimegui.append(', *')
                    runtimegui.append(midVar)
                    runtimegui.append(', ')
                    runtimegui.append(str(tran.id))
                    runtimegui.append(', ')
                    runtimegui.append(str(tran.origin.id))
                    runtimegui.append(', ')
                    runtimegui.append(str(tran.destination.id))
                    runtimegui.append(');\n')

            runtimegui.append('\t')
            runtimegui.append('guiSubautomataList.push_back(*')
            runtimegui.append(stateVar)
            runtimegui.append(');\n')

        runtimegui.append('\treturn guiSubautomataList;\n')
        runtimegui.append('}\n\n')

        return runtimegui

    def generateStates(self, stateStr):
        nameTimeMap = {}
        for state in self.states:
            stateStr.append('void* state_')
            stateStr.append(str(state.id))
            stateStr.append(' (void*) {\n')
            stateStr.append('\tstruct timeval a, b;\n')
            stateStr.append('\tint cycle = ')
            stateStr.append(str(state.getTimeStep()))
            stateStr.append(';\n')
            stateStr.append('\tlong totala, totalb;\n')
            stateStr.append('\tlong diff;\n')
            stateStr.append('\ttime_t t_ini;\n')
            stateStr.append('\ttime_t t_fin;\n')
            stateStr.append('\tdouble secs;\n')
            stateStr.append('\tbool t_activated = false;\n\n')

            # create time based transitions
            # for tran in state.getChildrenTransitions():
            #     if tran.getType() == TransitionType.TEMPORAL:
            #         stateStr.append('\tfloat t_')
            #         stateStr.append(self.sanitizeVar(state.name))
            #         stateStr.append('_max = ')
            #         stateStr.append(str(tran.getTemporalTime()/1000.0))
            #         stateStr.append(';\n')

            # variables
            varLines = state.getVariables().split('\n')
            for varLine in varLines:
                stateStr.append('\t')
                stateStr.append(varLine.strip('\n'))
                stateStr.append('\n')

            # step loop of the state
            stateStr.append('\twhile (run')
            stateStr.append(str(state.id))
            stateStr.append(') {\n')
            stateStr.append('\t\tgettimeofday(&a, NULL);\n')
            stateStr.append('\t\ttotala = a.tv_sec * 1000000 + a.tv_usec;\n\n')

            # if state.parent is not None:
            #     stateStr.append('\t\tif (sub_')
            #     stateStr.append(str(state.parent.id))
            #     stateStr.append(' == ')
            #     stateStr.append(self.sanitizeVar(state.parent.name))
            #     stateStr.append(') {\n')
            #
            #     stateStr.append('\t\t\tif (')
            #     for childState in state.getChildren():
            #         stateStr.append(' sub_')
            #         stateStr.append(str(state.id))
            #         stateStr.append(' == ')
            #         stateStr.append(self.sanitizeVar(childState.name))
            #         stateStr.append('_ghost')
            #         if childState != state.getChildren()[len(state.getChildren())-1]:
            #             stateStr.append(' || ')
            #
            #     stateStr.append(') {\n')
            #
            #     stateStr.append('\t\t\t\tsub_')
            #     stateStr.append(str(state.id))
            #     stateStr.append(' = (State_Sub_')
            #     stateStr.append(str(state.id))
            #     stateStr.append(')(sub_')
            #     stateStr.append(str(state.id))
            #     stateStr.append(' - 1);\n')
            #     stateStr.append('\t\t\t\tt_ini = time(NULL);\n')
            #     stateStr.append('\t\t\t}\n')

            stateStr.append('\t\t//Evaluation switch\n')
            stateStr.append('\t\tswitch (sub_')
            stateStr.append(str(state.id))
            stateStr.append(') {\n')
            for childState in state.getChildren():
                stateStr.append('\t\t\tcase ')
                stateStr.append(self.sanitizeVar(childState.name))
                stateStr.append(': {\n')

                for tran in childState.getOriginTransitions():
                    if tran.getType() == TransitionType.CONDITIONAL:
                        stateStr.append('\t\t\t\tif (')
                        stateStr.append(tran.getCondition())
                        stateStr.append(') {\n')
                        stateStr.append('\t\t\t\t\tsub_')
                        stateStr.append(str(state.id))
                        stateStr.append(' = ')
                        stateStr.append(self.sanitizeVar(tran.destination.name))
                        stateStr.append(';\n')
                        tranCode = tran.getCode()
                        for codeLine in tranCode.split('\n'):
                            stateStr.append('\t\t\t\t\t\t')
                            stateStr.append(codeLine)
                            stateStr.append('\n')
                        stateStr.append('\t\t\t\t\tif (displayGui) {\n')
                        stateStr.append('\t\t\t\t\t\tautomatagui->notifySetNodeAsActive("')
                        stateStr.append(tran.destination.name)
                        stateStr.append('");\n')
                        stateStr.append('\t\t\t\t\t}\n')
                        stateStr.append('\t\t\t\t}\n')
                    else:
                        stateStr.append('\t\t\t\tif (!t_activated) {\n')
                        stateStr.append('\t\t\t\t\tt_ini = time(NULL);\n')
                        stateStr.append('\t\t\t\t\tt_activated = true;\n')
                        stateStr.append('\t\t\t\t} else {\n')
                        stateStr.append('\t\t\t\t\tt_fin = time(NULL);\n')
                        stateStr.append('\t\t\t\t\tsecs = difftime(t_fin, t_ini);\n')
                        # if state.parent is None:
                        stateStr.append('\t\t\t\t\tif (secs > (double) ')
                        stateStr.append(str(tran.getTemporalTime() / 1000.0))
                        stateStr.append(') {\n')
                        # else:
                        #     stateStr.append('\t\t\t\t\tif (secs > (double) t_')
                        #     stateStr.append(self.sanitizeVar(state.name))
                        #     stateStr.append('_max) {\n')
                        stateStr.append('\t\t\t\t\t\tsub_')
                        stateStr.append(str(state.id))
                        stateStr.append(' = ')
                        stateStr.append(self.sanitizeVar(tran.destination.name))
                        stateStr.append(';\n')
                        stateStr.append('\t\t\t\t\t\tt_activated = false;\n')
                        tranCode = tran.getCode()
                        for codeLine in tranCode.split('\n'):
                            stateStr.append('\t\t\t\t\t\t')
                            stateStr.append(codeLine)
                            stateStr.append('\n')
                        stateStr.append('\t\t\t\t\t\tif (displayGui) {\n')
                        stateStr.append('\t\t\t\t\t\t\tautomatagui->notifySetNodeAsActive("')
                        stateStr.append(tran.destination.name)
                        stateStr.append('");\n')
                        stateStr.append('\t\t\t\t\t\t}\n')
                        # if state.parent is not None:
                        #     stateStr.append('\t\t\t\t\t\tt_')
                        #     stateStr.append(self.sanitizeVar(state.name))
                        #     stateStr.append('_max = ')
                        #     nameTimeMap[state.name] = tran.getTemporalTime() / 1000.0
                        #     stateStr.append(str(tran.getTemporalTime() / 1000.0))
                        #     stateStr.append(';\n')
                        stateStr.append('\t\t\t\t\t}\n')
                        stateStr.append('\t\t\t\t}\n')
                    stateStr.append('\n')

                stateStr.append('\t\t\t\tbreak;\n')
                stateStr.append('\t\t\t}\n')

            stateStr.append('\t\t}\n\n')

            stateStr.append('\t\t// Actuation switch\n')
            stateStr.append('\t\tswitch (sub_')
            stateStr.append(str(state.id))
            stateStr.append(') {\n')
            for childState in state.getChildren():
                stateStr.append('\t\t\tcase ')
                stateStr.append(self.sanitizeVar(childState.name))
                stateStr.append(': {\n')
                for codeLine in childState.getCode().split('\n'):
                    stateStr.append('\t\t\t\t')
                    stateStr.append(codeLine)
                    stateStr.append('\n')
                stateStr.append('\t\t\t\tbreak;\n')
                stateStr.append('\t\t\t}\n')

            stateStr.append('\t\t}\n')

            # if state.parent is not None:
            #     stateStr.append('\t\t} else {\n')
            #     stateStr.append('\t\t\tswitch (sub_')
            #     stateStr.append(str(state.id))
            #     stateStr.append(') {\n')
            #     for childState in state.getChildren():
            #         stateStr.append('\t\t\t\tcase ')
            #         stateStr.append(self.sanitizeVar(childState.name))
            #         stateStr.append(':\n')
            #         if childState.name in nameTimeMap:
            #             stateStr.append('\t\t\t\t\tt_')
            #             stateStr.append(self.sanitizeVar(childState.name))
            #             stateStr.append('_max = ')
            #             stateStr.append(str(nameTimeMap[childState.name]))
            #             stateStr.append(' - difftime(t_fin, t_ini);\n')
            #         stateStr.append('\t\t\t\t\tsub_')
            #         stateStr.append(str(state.id))
            #         stateStr.append(' = (State_Sub_')
            #         stateStr.append(str(state.id))
            #         stateStr.append(')(sub_);\n')
            #         stateStr.append('\t\t\t\t\tbreak;\n')
            #     stateStr.append('\t\t\t\tdefault:\n')
            #     stateStr.append('\t\t\t\t\tbreak;\n')
            #     stateStr.append('\t\t\t}\n')
            #     stateStr.append('\t\t}\n')

            stateStr.append('\n')

            stateStr.append('\t\tgettimeofday(&b, NULL);\n')
            stateStr.append('\t\ttotalb = b.tv_sec * 1000000 + b.tv_usec;\n')
            stateStr.append('\t\tdiff = (totalb - totala) / 1000;\n')
            stateStr.append('\t\tif (diff < 0 || diff > cycle)\n')
            stateStr.append('\t\t\tdiff = cycle;\n')
            stateStr.append('\t\telse\n')
            stateStr.append('\t\t\tdiff = cycle - diff;\n\n')
            stateStr.append('\t\tusleep(diff * 1000);\n')
            stateStr.append('\t\tif (diff < 33)\n')
            stateStr.append('\t\t\tusleep(33*1000);\n')

            stateStr.append('\t}\n')
            stateStr.append('}\n\n')

        return stateStr

    def generateAutomataGui(self, guiStr):
        guiStr.append('void* runAutomatagui(void*) {\n')
        guiStr.append('\tautomatagui->run();\n')
        guiStr.append('}\n\n')

        guiStr.append('bool showAutomataGui() {\n')
        guiStr.append('\tif (automatagui->init() < 0) {\n')
        guiStr.append('\t\tstd::cerr << "warning: could not show automatagui" << std::endl;\n')
        guiStr.append('\t\treturn false;\n')
        guiStr.append('\t}\n')

        guiStr.append('\tautomatagui->setGuiSubautomataList(createGuiSubAutomataList());\n')
        guiStr.append('\tpthread_create(&thr_gui, NULL, runAutomatagui, NULL);\n')
        guiStr.append('\tautomatagui->loadGuiSubautomata();\n')
        guiStr.append('\treturn true;\n')
        guiStr.append('}\n\n')

        return guiStr

    def generateReadArgs(self, readArgsStr):
        mystr = '''
void readArgs(int *argc, char* argv[]){
    int i;
    std::string splitedArg;
    
    for(i = 0; i < *argc; i++){
        splitedArg = strtok(argv[i], "=");
        if (splitedArg.compare("--displaygui") == 0){
            splitedArg = strtok(NULL, "=");
            if (splitedArg.compare("true") == 0 || splitedArg.compare("True") == 0){
                displayGui = true;
                std::cout << "displayGui ENABLED" << std::endl;
            }else{
                displayGui = false;
                std::cout << "displayGui DISABLED" << std::endl;
            }
        }
        if(i == *argc -1){
            (*argc)--;
        }
    }
}

'''

        readArgsStr.append(mystr)
        return readArgsStr

    def generateMain(self, mainStr):
        mainStr.append('int main(int argc, char* argv[]) {\n')
        mainStr.append('\tint status;\n\n')
        mainStr.append('\tIce::CommunicatorPtr ic;\n')
        mainStr.append('\ttry {\n')
        mainStr.append('\t\tic = EasyIce::initialize(argc, argv);\n')
        mainStr.append('\t\treadArgs(&argc, argv);\n\n')

        for cfg in self.configs:
            mainStr.append('\t\t// Contact to ')
            mainStr.append(cfg['name'])
            mainStr.append('\n\t\tIce::ObjectPrx temp_')
            mainStr.append(cfg['name'])
            mainStr.append(' = ic->propertyToProxy("automata.')
            mainStr.append(cfg['name'])
            mainStr.append('.Proxy");\n')
            mainStr.append('\t\tif (temp_')
            mainStr.append(cfg['name'])
            mainStr.append(' == 0)\n')
            mainStr.append('\t\t\tthrow "Could not create proxy with ')
            mainStr.append(cfg['name'])
            mainStr.append('";\n')
            mainStr.append('\t\t')
            mainStr.append(cfg['name'])
            mainStr.append(' = jderobot::')
            mainStr.append(cfg['interface'])
            mainStr.append('Prx::checkedCast(temp_')
            mainStr.append(cfg['name'])
            mainStr.append(');\n')
            mainStr.append('\t\tif (')
            mainStr.append(cfg['name'])
            mainStr.append(' == 0)\n')
            mainStr.append('\t\t\tthrow "Invalid proxy automata.')
            mainStr.append(cfg['name'])
            mainStr.append('.Proxy";\n')
            mainStr.append('\t\tstd::cout << "')
            mainStr.append(cfg['name'])
            mainStr.append(' connected" << std::endl;\n\n')

        mystr = '''
        if (displayGui) {
            automatagui = new AutomataGui(argc, argv);
            displayGui = showAutomataGui();
        }
'''
        mainStr.append(mystr)
        mainStr.append('\n')

        for state in self.states:
            mainStr.append('\t\tpthread_create(&thr_')
            mainStr.append(str(state.id))
            mainStr.append(', NULL, &state_')
            mainStr.append(str(state.id))
            mainStr.append(', NULL);\n')

        for state in self.states:
            mainStr.append('\t\tpthread_join(thr_')
            mainStr.append(str(state.id))
            mainStr.append(', NULL);\n')

        mystr = '''
        if (displayGui)
            pthread_join(thr_gui, NULL);
'''
        mainStr.append(mystr)
        mainStr.append('\n')

        mystr = '''
    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        status = 1;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        status = 1;
    }
            
    if (ic)
        ic->destroy();
                
    return status;
}
'''
        mainStr.append(mystr)


    def generateCmake(self, cmakeStr, projectName):
        cmakeStr.append('project(')
        cmakeStr.append(projectName)
        cmakeStr.append(')\n\n')

        cmakeStr.append('cmake_minimum_required(VERSION 2.8)\n')
        cmakeStr.append('include(FindPkgConfig)\n\n')

        cmakeStr.append('SET(SOURCE_FILES_AUTOMATA\n')
        cmakeStr.append('\t')
        cmakeStr.append(projectName)
        cmakeStr.append('.cpp')
        cmakeStr.append(')\n\n')

        mystr = '''
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
        
SET(INTERFACES_CPP_DIR /opt/jderobot/include)
SET(LIBS_DIR /usr/local/lib)
SET(JDEROBOT_LIBS_DIR /opt/jderobot/lib)
        
SET(CMAKE_CXX_FLAGS "-pthread")
        
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
        
SET(goocanvasmm_INCLUDE_DIRS
    \t/usr/include/goocanvasmm-2.0
    \t/usr/lib/goocanvasmm-2.0/include
    \t/usr/include/goocanvas-2.0
)
        
include_directories(
    \t/opt/jderobot/include/
    \t${INTERFACES_CPP_DIR}
    \t${easyiceconfig_INCLUDE_DIRS}
    \t${LIBS_DIR}
    \t${JDEROBOT_LIBS_DIR}
    \t${CMAKE_CURRENT_SOURCE_DIR}
    \t${GTKMM_INCLUDE_DIRS}
    \t${goocanvasmm_INCLUDE_DIRS}
)
        
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
        
link_directories(${GTKMM_LIBRARY_DIRS} ${JDEROBOT_LIBS_DIR})
'''
        cmakeStr.append(mystr)
        #todo: how to get name of the project
        cmakeStr.append('add_executable(')
        cmakeStr.append(projectName)
        cmakeStr.append(' ${SOURCE_FILES_AUTOMATA})\n\n')
        cmakeStr.append('SET(goocanvasmm_LIBRARIES goocanvasmm-2.0 goocanvas-2.0)\n\n')

        cmakeStr.append('TARGET_LINK_LIBRARIES (\t')
        cmakeStr.append(projectName)
        cmakeStr.append('\n')

        mystr = '''
    ${GTKMM_LIBRARIES}
    easyiceconfig
    ${goocanvasmm_LIBRARIES}
    visualStateslib
    JderobotInterfaces
    jderobotutil
    Ice
    IceUtil
)
        
'''
        cmakeStr.append(mystr)

        return cmakeStr











































































