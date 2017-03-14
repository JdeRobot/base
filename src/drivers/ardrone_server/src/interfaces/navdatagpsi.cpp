/*
 *  Copyright (C) 1997-2015 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *  Authors :
 *       Victor Arribas Raigadas <.varribas.urjc@gmail.com>
 */

#include "ardrone_server/interfaces/navdatagpsi.h"


using namespace ardrone_server::interfaces;


NavdataGPSI::NavdataGPSI()
{
    std::cout << "navdatagps start" << std::endl;
    memset(&gpsdata_defaults_, 0, sizeof(ardrone::NavdataGPSData));
    gpsdata_defaults_.channels = ardrone::Satellites(ardrone::MaxNumChannels);
    assert(sizeof(ardrone::SatelliteInfo) == sizeof(uint8_t)*2);
}

NavdataGPSI::~NavdataGPSI()
{
    std::cout << "navdatagps end" << std::endl;
}

ardrone::NavdataGPSData NavdataGPSI::getNavdataGPS(Ice::Current const & /*c*/)
{
    vp_os_mutex_lock(&navdata_lock);
            navdata_unpacked_t navdata_raw = *shared_raw_navdata;
    vp_os_mutex_unlock(&navdata_lock);

    ardrone::NavdataGPSData gpsdata = gpsdata_defaults_;

    gpsdata.tag = navdata_raw.navdata_gps_info.tag;
    gpsdata.size = navdata_raw.navdata_gps_info.size;
    gpsdata.latitude = navdata_raw.navdata_gps_info.latitude;
    gpsdata.longitude = navdata_raw.navdata_gps_info.longitude;
    gpsdata.elevation = navdata_raw.navdata_gps_info.elevation;
    gpsdata.hdop = navdata_raw.navdata_gps_info.hdop;
    gpsdata.dataAvailable = navdata_raw.navdata_gps_info.data_available;
    gpsdata.zeroValidated = navdata_raw.navdata_gps_info.zero_validated;
    gpsdata.wptValidated = navdata_raw.navdata_gps_info.wpt_validated;
    gpsdata.lat0 = navdata_raw.navdata_gps_info.lat0;
    gpsdata.long0 = navdata_raw.navdata_gps_info.long0;
    gpsdata.latFused = navdata_raw.navdata_gps_info.lat_fused;
    gpsdata.longFused = navdata_raw.navdata_gps_info.long_fused;
    gpsdata.gpsState = navdata_raw.navdata_gps_info.gps_state;
    gpsdata.Xtraj = navdata_raw.navdata_gps_info.X_traj;
    gpsdata.Xref = navdata_raw.navdata_gps_info.X_ref;
    gpsdata.Ytraj = navdata_raw.navdata_gps_info.Y_traj;
    gpsdata.Yref = navdata_raw.navdata_gps_info.Y_ref;
    gpsdata.thetaP = navdata_raw.navdata_gps_info.theta_p;
    gpsdata.phiP = navdata_raw.navdata_gps_info.phi_p;
    gpsdata.thetaI = navdata_raw.navdata_gps_info.theta_i;
    gpsdata.phiI = navdata_raw.navdata_gps_info.phi_i;
    gpsdata.thetaD = navdata_raw.navdata_gps_info.theta_d;
    gpsdata.phiD = navdata_raw.navdata_gps_info.phi_d;
    gpsdata.vdop = navdata_raw.navdata_gps_info.vdop;
    gpsdata.pdop = navdata_raw.navdata_gps_info.pdop;
    gpsdata.speed = navdata_raw.navdata_gps_info.speed;
    gpsdata.lastFrameTimestamp = navdata_raw.navdata_gps_info.lastFrameTimestamp;
    gpsdata.degree = navdata_raw.navdata_gps_info.degree;
    gpsdata.degreeMagnetic = navdata_raw.navdata_gps_info.degree_magnetic;
    gpsdata.ehpe = navdata_raw.navdata_gps_info.ehpe;
    gpsdata.ehve = navdata_raw.navdata_gps_info.ehve;
    gpsdata.channelsNoiseRatio = navdata_raw.navdata_gps_info.c_n0;
    gpsdata.nbsat = navdata_raw.navdata_gps_info.nbsat;
    memcpy(gpsdata.channels.data(), &(navdata_raw.navdata_gps_info.channels[0]), ardrone::MaxNumChannels*sizeof(ardrone::SatelliteInfo)); //TODO: keep an eye on it!
    gpsdata.isGpsPlugged = navdata_raw.navdata_gps_info.is_gps_plugged;
    gpsdata.ephemerisStatus = navdata_raw.navdata_gps_info.ephemerisStatus;
    gpsdata.vxTraj = navdata_raw.navdata_gps_info.vx_traj;
    gpsdata.vyTraj = navdata_raw.navdata_gps_info.vy_traj;
    gpsdata.firmwareStatus = navdata_raw.navdata_gps_info.firmwareStatus;

//    for (int i=0; i<jderobot::MaxNumChannels; i++){
//        gpsdata.channels[i].satID = navdata_raw.navdata_gps_info.channels[i].sat;
//        gpsdata.channels[i].carrierToNoiseRatio = navdata_raw.navdata_gps_info.channels[i].c_n0;
//    }


    return gpsdata;

}
