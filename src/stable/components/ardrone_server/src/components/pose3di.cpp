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

	#define deg2rad(x) (PI*(x)/180.0)    /* from grades to radians */
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

namespace {
	typedef Eigen::Quaternion<float> quat;
	Eigen::Quaternion<float> qFromRPY(float roll, float pitch, float yaw){
		quat q;
		Eigen::AngleAxis<float> aaZ( yaw,   Eigen::Vector3f::UnitZ());
		Eigen::AngleAxis<float> aaY( pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxis<float> aaX( roll,  Eigen::Vector3f::UnitX());
		q = aaZ * aaY * aaX;
		return q;
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
		xyzT = {0,0,0};
		qR.setIdentity();
	}
	
	Pose3DI::~Pose3DI()
	{

	}
		
	jderobot::Pose3DDataPtr Pose3DI::getPose3DData(const Ice::Current&)
	{
		vp_os_mutex_lock(&navdata_lock);
			navdata_unpacked_t navdata_raw = *shared_raw_navdata;
		vp_os_mutex_unlock(&navdata_lock);	

		pose3d_t xyz = {0,0,0};
		pose3d_t angles = {0,0,0};

		/// Get orientation
		angles.roll  = deg2rad( navdata_raw.navdata_demo.phi   / 1000.0);
		angles.pitch = deg2rad(-navdata_raw.navdata_demo.theta / 1000.0);
		angles.yaw   = deg2rad(-navdata_raw.navdata_demo.psi   / 1000.0);

		/// Get GPS position
		bool is_gps_plugged     = navdata_raw.navdata_gps_info.is_gps_plugged;
		uint32_t firmwareStatus = navdata_raw.navdata_gps_info.firmwareStatus;
		uint32_t gps_state      = navdata_raw.navdata_gps_info.gps_state;
		float64_t latitude  = navdata_raw.navdata_gps_info.latitude;
		float64_t longitude = navdata_raw.navdata_gps_info.longitude;
		float64_t elevation = navdata_raw.navdata_gps_info.elevation;

		/// Check GPS status
		gps_on = (is_gps_plugged && firmwareStatus == 1);
		gps_valid = (gps_state == 1);

		if (gps_on && gps_valid){
			spherical2cartesian(latitude, longitude, elevation, xyz.x, xyz.y, xyz.z);

			if (!gps_is_bootstrapped){
				gps_is_bootstrapped = true;
				printf("GPS boostrap at %f, %f, %f\n", latitude, longitude, elevation);

				poseBoostrapWithExternals(xyz, angles);
			}
		}
		if (!gps_on){
			if (gps_is_bootstrapped){
				gps_is_bootstrapped = false;
				printf("GPS disconnected\n");
			}
		}

		/// Origin base change
		//Rotation
		Eigen::Quaternion<float> q = qFromRPY(angles.roll, angles.pitch, angles.yaw);
		q = qR*q;
		// Translation
		Point3D_OP(xyz, xyz, +, xyzT);

		/// Push values
		pose3D->q0=q.w();
		pose3D->q1=q.x();
		pose3D->q2=q.y();
		pose3D->q3=q.z();

		pose3D->x = xyz.x;
		pose3D->y = xyz.y;
		pose3D->z = xyz.z;
		pose3D->h = gps_valid? 1 : 0;

		return pose3D;
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


	void Pose3DI::poseBoostrapWithExternals(pose3d_t current_xyz, pose3d_t current_angles)
	{
		pose3d_t diff, target;
		//// Compose Translation
		/// Default behavior must be T={0,0,0}
		target.x = driver->getParameter("origin_x", current_xyz.x);
		target.y = driver->getParameter("origin_y", current_xyz.y);
		target.z = driver->getParameter("origin_z", current_xyz.z);
		Point3D_OP(diff, target, -, current_xyz);
		xyzT = diff;

		//// Compose Rotation
		/// Default behavior must be identity
		target.roll  = current_angles.roll  + deg2rad(driver->getParameter("origin_roll",  0));
		target.pitch = current_angles.pitch + deg2rad(driver->getParameter("origin_pitch", 0));
		target.yaw   = deg2rad(driver->getParameter("origin_yaw", current_angles.yaw));
		Point3D_OP(diff, target, -, current_angles);
		qR = qFromRPY(diff.roll, diff.pitch, diff.yaw);

#if 0 // for test purposes only
		quat q_t = qFromRPY(target.roll, target.pitch, target.yaw);
		quat q_c = qFromRPY(current_angles.roll, current_angles.pitch, current_angles.yaw);
		quat qR2 = q_t*q_c.inverse();

		printf("Pose3D boostrap with transforms.\n");
		printf("\t XYZ transform: %.2f, %.2f, %.2f\n",
           -xyzT.x, -xyzT.y, -xyzT.z);
		printf("\t Orientation transform: %.4f, %.4f, %.4f\n",
           -diff.x, -diff.y, -diff.z);

		printf("qR:  %.2f, %.2f, %.2f, %.2f\n", qR.x(), qR.y(), qR.z(), qR.w());
		printf("qR2: %.2f, %.2f, %.2f, %.2f\n", qR2.x(), qR2.y(), qR2.z(), qR2.w());
#endif
	}
}
