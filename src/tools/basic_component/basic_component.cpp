#include "gui/threadupdategui.h"
#include "parallelIce/cameraClient.h"
#include "parallelIce/motorsClient.h"
#include "easyiceconfig/EasyIce.h"


int main(int argc, char* argv[])
{

    QApplication app(argc, argv);

    int status;
    Ice::CommunicatorPtr ic;
    jderobot::cameraClient* camera;
    jderobot::motorsClient* motors;

    try{
        ic = EasyIce::initialize(argc,argv);

        camera = new jderobot::cameraClient(ic,"basic_component.Camera.");
        camera->start();

        motors = new jderobot::motorsClient(ic, "basic_component.Motors.");

        ThreadUpdateGUI* thread_update_gui = new ThreadUpdateGUI(ic, camera, motors);
        thread_update_gui->start();

    }catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        status = 1;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        status = 1;
    }


    app.exec();

    if (ic)
        ic->destroy();

    return status;
}