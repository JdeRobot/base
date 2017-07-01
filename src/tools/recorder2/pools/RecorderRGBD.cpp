//
// Created by frivas on 21/05/17.
//

#include <jderobotutil/utils/CameraUtils.h>
#include "RecorderRGBD.h"


namespace recorder {
    RecorderRGBD::RecorderRGBD(const jderobot::rgbData &data) {
        rgb = CameraUtils::getImageFromCameraProxy(data.color);
        depth = CameraUtils::getImageFromCameraProxy(data.depth);
    }

    RecorderRGBD::RecorderRGBD(const RecorderRGBD &other) {
        rgb= other.rgb.clone();
        depth = other.depth.clone();
    }

    RecorderRGBD::RecorderRGBD() {

    }
}