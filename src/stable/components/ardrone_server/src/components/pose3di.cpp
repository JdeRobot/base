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
 *       Alberto Martín Florido <almartinflorido@gmail.com>
 *       Victor Arribas Raigadas <v.arribas.urjc@gmai.com>
 */
#include "pose3di.h"
#include <cmath>


namespace {
	/// Weak cartesian calculation based on Spherical Coordinates.
	/// See:
	/// * https://en.wikipedia.org/wiki/Spherical_coordinate_system
	/// * https://en.wikipedia.org/wiki/Flattening
	/// * http://gis.stackexchange.com/questions/23793/how-do-i-calculate-a-xyz-position-of-a-gps-position-relative-to-an-other-gps-pos#
	/// * http://geographiclib.sourceforge.net/cgi-bin/GeodSolve

	#define deg2rad(x) (PI*x/180.0)    /* from grades to radians */
	#define wgs84_radius 6378137
	#define wgs84_flattening (1 - 1/298.257223563)

	void spherical2cartesian(double lat, double lon, double alt, double &x, double &y, double &z){
		lat = deg2rad(lat);
		lon = deg2rad(lon);
		double r = wgs84_radius + alt;
		x = r*cos(lat)*cos(lon);
		y = r*cos(lat)*sin(lon);
		z = r*sin(lat)*wgs84_flattening;
	}
}


namespace pose3D
{
	Pose3DI::Pose3DI(ARDroneDriver *driver):
        driver(driver),
        gps_on(false), gps_valid(false), gps_is_bootstrapped(false)
	{
		std::cout << "pose3d start" << std::endl;
		pose3D = new jderobot::Pose3DData(0,0,0,0, 0,0,0,0);
	}
	
	Pose3DI::~Pose3DI()
	{

	}
		
	jderobot::Pose3DDataPtr Pose3DI::getPose3DData(const Ice::Current&)
	{
		vp_os_mutex_lock(&navdata_lock);
			navdata_unpacked_t navdata_raw = *shared_raw_navdata;
		vp_os_mutex_unlock(&navdata_lock);	

		/// Push orientation
		float roll=navdata_raw.navdata_demo.phi / 1000.0;
		float pitch=-navdata_raw.navdata_demo.theta / 1000.0;
		float yaw=-navdata_raw.navdata_demo.psi / 1000.0;

		Eigen::Quaternion<float> q;
		Eigen::AngleAxis<float> aaZ(this->deg2rad(yaw), Eigen::Vector3f::UnitZ());
		Eigen::AngleAxis<float> aaY(this->deg2rad(pitch), Eigen::Vector3f::UnitY());
		Eigen::AngleAxis<float> aaX(this->deg2rad(roll), Eigen::Vector3f::UnitX());

		q = aaZ * aaY * aaX;	
		
		pose3D->q0=q.w();
		pose3D->q1=q.x();
		pose3D->q2=q.y();
		pose3D->q3=q.z();


		///Push position
		bool is_gps_plugged = navdata_raw.navdata_gps_info.is_gps_plugged;
		uint32_t firmwareStatus = navdata_raw.navdata_gps_info.firmwareStatus;
		float64_t latitude = navdata_raw.navdata_gps_info.latitude;
		float64_t longitude = navdata_raw.navdata_gps_info.longitude;
		float64_t elevation = navdata_raw.navdata_gps_info.elevation;
		uint32_t gps_state = navdata_raw.navdata_gps_info.gps_state;

		gps_on = (is_gps_plugged && firmwareStatus == 1);
		gps_valid = (gps_state == 1);

		if (gps_on && gps_valid){
			if (!gps_is_bootstrapped){
				spherical2cartesian(latitude, longitude, elevation, x0, y0, z0);
				gps_is_bootstrapped = true;
				printf("GPS boostrap at %f, %f, %f\n", latitude, longitude, elevation);
			}

			double x,y,z;
			spherical2cartesian(latitude, longitude, elevation, x,y,z);
			x -= x0;
			y -= y0;
			z -= z0;

			pose3D->x = x;
			pose3D->y = y;
			pose3D->z = z;
			pose3D->h = 1;
		}

		if (!gps_valid)
			pose3D->h = 0;

		if (!gps_on){
			pose3D->h = 0;
			if (gps_is_bootstrapped){
				gps_is_bootstrapped = false;
				printf("GPS disconnected\n");
			}
		}


		return pose3D;
	}
	
	float Pose3DI::deg2rad(float d)
	{
		float radians = (d / 360) * (2.0 * PI);
		return radians;
	}
	
	Ice::Int Pose3DI::setPose3DData(const jderobot::Pose3DDataPtr& data, const Ice::Current&)
	{
	
		pose3D->x=data->x;
		pose3D->y=data->y;
		pose3D->z=data->z;
		pose3D->h=data->h;
		pose3D->q0=data->q0;
		pose3D->q1=data->q1;
		pose3D->q2=data->q2;
		pose3D->q3=data->q3;														
		return 1;
	}
}
