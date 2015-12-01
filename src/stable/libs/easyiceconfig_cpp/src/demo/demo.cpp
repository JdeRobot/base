/*
 * Copyright (c) 2015
 * Author: Victor Arribas <v.arribas.urjc@gmail.com>
 * License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>
 */

#include <Ice/Ice.h>

using namespace std;

int
main(int argc, char* argv[])
{
    Ice::CommunicatorPtr ic;

    //// Instance ic from properties
    /// this allows:
    ///  * multiple config files load
    ///  * config from command line
    ///  * in-code changes
    Ice::PropertiesPtr properties;
#if 1
    /// From empty properties we can only
    /// obtain direct configurations
    ///  * yes: --X=Y
    ///  * no: content of --Ice.Config
    properties = Ice::createProperties();
    Ice::StringSeq args(argv+1, argv+argc);
    properties->parseIceCommandLineOptions(args);
#else
    /// Same load as Ice::initialize
    /// parse args + config file
    properties = Ice::createProperties(argc, argv);
    //id.properties = Ice::createProperties(args);
#endif

    /// Append/override from additional config file
    properties->load("ice.cfg");

    /// Programatic modifications
    properties->setProperty("Demo.Endpoints", "-666");
    {
    Ice::StringSeq args(argv+1, argv+argc);
    properties->parseIceCommandLineOptions(args);
    }

    /// Communicator initialize
    Ice::InitializationData id;
    id.properties = properties;
    ic = Ice::initialize(id);


    //// Print properties
    Ice::PropertyDict dict = ic->getProperties()->getPropertiesForPrefix("");
    std::cout<<"Num keys: "<<dict.size()<<std::endl;
    std::map<std::string,std::string>::iterator at;
    for (at=dict.begin(); at!=dict.end(); at++){
	  std::cout<<"\t"<<at->first<<": "<<at->second<<std::endl;
    }

    return 0;
}
