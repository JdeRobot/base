#include "gui/threadupdategui.h"
#include <jderobot/config/config.h> 
#include <jderobot/comm/communicator.hpp>
#include <jderobot/comm/cameraClient.hpp>
#include <jderobot/comm/motorsClient.hpp>


int main(int argc, char* argv[])
{

    QApplication app(argc, argv);

    int status;
    Comm::Communicator* jdrc;
    Comm::CameraClient* camera;
    Comm::MotorsClient* motors;


    try{

        Config::Properties cfg = Config::load(argc, argv);
        jdrc = new Comm::Communicator(cfg);

        camera = Comm::getCameraClient(jdrc, "basic_component.Camera");

        motors = Comm::getMotorsClient(jdrc, "basic_component.Motors");

        ThreadUpdateGUI* thread_update_gui = new ThreadUpdateGUI(jdrc, camera, motors);
        thread_update_gui->start();

    }catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        status = 1;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        status = 1;
    }


    app.exec();

    delete jdrc;

    return status;
}