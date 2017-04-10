//
// Created by frivas on 25/01/17.
//

#include "PointCloudServer.h"


namespace openniServer {

    PointCloudServer::PointCloudServer(std::string &propertyPrefix, const Ice::PropertiesPtr propIn,ConcurrentDevicePtr device) :
            prefix(propertyPrefix),
            data(new jderobot::pointCloudData()),
            device(device)
    {
        Ice::PropertiesPtr prop = propIn;

        int fps = prop->getPropertyAsIntWithDefault("openniServer.pointCloud.Fps", 10);
        bool extra = (bool) prop->getPropertyAsIntWithDefault("openniServer.ExtraCalibration", 0);
        replyCloud = new ReplyCloud(this, device, prop->getProperty("openniServer.calibration"), fps, extra);
        this->control = replyCloud->start();
    }

    PointCloudServer::~PointCloudServer() {
        LOG(INFO) << "Stopping and joining thread for pointCloud";
        replyCloud->destroy();
        this->control.join();

    }

    jderobot::pointCloudDataPtr PointCloudServer::getCloudData(const Ice::Current &) {
        data = replyCloud->getCloud();
        return data;
    }

    PointCloudServer::ReplyCloud::ReplyCloud(PointCloudServer *pcloud, ConcurrentDevicePtr device, std::string calibration_filepath, int fpsIn, bool extra_calibration):
            myCloud(pcloud),
            device(device),
            mypro(NULL),
            calibration_filepath(calibration_filepath),
            fps(fpsIn),
            temporalData(new jderobot::pointCloudData()),
            stableData(new jderobot::pointCloudData()),
            returnData(new jderobot::pointCloudData()),
            withExtraCalibration(extra_calibration),
            _done(false)
    {
        this->cameraSize=cv::Size(device->getVideoMode().witdh,device->getVideoMode().heigth);
    }

    void PointCloudServer::ReplyCloud::run() {
        mypro = new openniServer::myprogeo(1, this->cameraSize.width, this->cameraSize.height);
        mypro->load_cam((char *) this->calibration_filepath.c_str(), 0, this->cameraSize.width, this->cameraSize.height, withExtraCalibration);


        int cycle; // duración del ciclo


        cycle = (float) (1 / (float) fps) * 1000000;
        IceUtil::Time lastIT = IceUtil::Time::now();
        while (!(_done)) {
            float distance;
            //creamos una copia local de la imagen de color y de las distancias.
            cv::Mat localRGB = this->device->getRGBImage();

            std::vector<int> localDistance;
            this->device->getDistances(localDistance);

            int step;
            if (this->cameraSize.width<=320){
                step=9;
            }
            else{
                step=18;
            }


            temporalData->p.clear();
            for (unsigned int i = 0; (i < (unsigned int) (this->cameraSize.width * this->cameraSize.height)) && (localDistance.size() > 0); i +=step) {
                distance = (float) localDistance[i];
                if (distance != 0) {
                    float xp, yp, zp, camx, camy, camz;
                    float ux, uy, uz;
                    float x, y;
                    float k;
                    float c1x, c1y, c1z;
                    float fx, fy, fz;
                    float fmod;
                    float t;
                    float Fx, Fy, Fz;

                    mypro->mybackproject(i % this->cameraSize.width, i / this->cameraSize.width, &xp, &yp, &zp, &camx, &camy, &camz, 0);

                    //vector unitario
                    float modulo;

                    modulo = sqrt(1 / (((camx - xp) * (camx - xp)) + ((camy - yp) * (camy - yp)) +
                                       ((camz - zp) * (camz - zp))));
                    mypro->mygetcamerafoa(&c1x, &c1y, &c1z, 0);

                    fmod = sqrt(1 / (((camx - c1x) * (camx - c1x)) + ((camy - c1y) * (camy - c1y)) +
                                     ((camz - c1z) * (camz - c1z))));
                    fx = (c1x - camx) * fmod;
                    fy = (c1y - camy) * fmod;
                    fz = (c1z - camz) * fmod;
                    ux = (xp - camx) * modulo;
                    uy = (yp - camy) * modulo;
                    uz = (zp - camz) * modulo;
                    Fx = distance * fx + camx;
                    Fy = distance * fy + camy;
                    Fz = distance * fz + camz;
                    // calculamos el punto real
                    t = (-(fx * camx) + (fx * Fx) - (fy * camy) + (fy * Fy) - (fz * camz) + (fz * Fz)) /
                        ((fx * ux) + (fy * uy) + (fz * uz));
                    auxP.x = t * ux + camx;
                    auxP.y = t * uy + camy;
                    auxP.z = t * uz + camz;


                    if (withExtraCalibration) {
                        mypro->applyExtraCalibration(&auxP.x, &auxP.y, &auxP.z);
                    }

                    if (localRGB.rows != 0) {
                        auxP.r = (float) (int) (unsigned char) localRGB.data[3 * i];
                        auxP.g = (float) (int) (unsigned char) localRGB.data[3 * i + 1];
                        auxP.b = (float) (int) (unsigned char) localRGB.data[3 * i + 2];
                    }
                    temporalData->p.push_back(auxP);
                }
                //}
            }

            this->mutex.lock();
            this->stableData->p= this->temporalData->p;
            this->mutex.unlock();

            int delay = IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds();
            if (delay > cycle) {
                LOG(WARNING) << "-------- openniServer: POINTCLOUD openni timeout-";
            } else {
                if (delay < 1 || delay > cycle)
                    delay = 1;
                usleep(delay);
            }


            lastIT = IceUtil::Time::now();
        }
    }

    jderobot::pointCloudDataPtr PointCloudServer::ReplyCloud::getCloud() {
        this->mutex.lock();
        this->returnData->p = this->stableData->p;
        this->mutex.unlock();
        return this->returnData;
    }

    void PointCloudServer::ReplyCloud::destroy() {
        this->_done = true;
    }

}