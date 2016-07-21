//
// Created by frivas on 8/12/15.
//

#ifndef JDEROBOT_COMMONHANDLER_H
#define JDEROBOT_COMMONHANDLER_H


#include <Ice/Communicator.h>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

namespace  jderobot {
    template<typename T>
    class CommonHandler {
    public:
        CommonHandler(const Ice::CommunicatorPtr &ic, const std::string &proxyStr, bool stringIsAlreadyProxy){
            if (!stringIsAlreadyProxy){
                try {
                    base = ic->propertyToProxy(proxyStr);
                    if (0 == base) {
                        throw "Could not create proxy to" + proxyStr;
                    }
                } catch (const Ice::Exception &ex) {
                    std::cerr << "Could not create proxy to" << proxyStr << ": " <<  ex << std::endl;
                }
                catch (const char *msg) {
                    std::cerr << "Could not create proxy to" << proxyStr << ": " <<  msg << std::endl;
                }
            }
            else{
                base = ic->stringToProxy(proxyStr);
            }

            if (base) {
                proxy = T::checkedCast(base);
            }

        }
        T getproxy(){
            return proxy;
        }

    private:
        T proxy;
        Ice::ObjectPrx base;
    };

}

#endif //JDEROBOT_COMMONHANDLER_H
