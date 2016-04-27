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

#ifndef ARDRONE_NAVDATAGPS_ICE
#define ARDRONE_NAVDATAGPS_ICE

/*
byte  8 bits
short 16 bits
int   32 bits
long  64 bits
*/

//module jderobot{
module ardrone {

    const byte MaxNumChannels = 12;

    struct SatelliteInfo{
        byte satID;                 /* Satellite ID */
        byte carrierToNoiseRatio;   /* Satellite C/N0 */
    };
    sequence<SatelliteInfo> Satellites;

    struct NavdataGPSData
    {
        short tag;
        short size;
        double latitude;
        double longitude;
        double elevation;
        double hdop;
        int   dataAvailable;
        bool zeroValidated;
        bool wptValidated;
        double lat0;
        double long0;
        double latFused;
        double longFused;

        int gpsState;

        float Xtraj;
        float Xref;
        float Ytraj;
        float Yref;

        float thetaP;
        float phiP;
        float thetaI;
        float phiI;
        float thetaD;
        float phiD;

        double vdop;
        double pdop;

        float speed;
        int  lastFrameTimestamp;
        float degree;
        float degreeMagnetic;
        float ehpe;
        float ehve;

        float channelsNoiseRatio;  /* Signal to noise ratio (average of the four best satellites) */
        int  nbsat;                /* Number of acquired satellites */
        Satellites channels;

        bool isGpsPlugged;
        int ephemerisStatus;

        float vxTraj;
        float vyTraj;


        int firmwareStatus;

    };

    interface NavdataGPS {
        idempotent NavdataGPSData getNavdataGPS();
    };


//}; //ardrone
}; //jderobot
#endif
