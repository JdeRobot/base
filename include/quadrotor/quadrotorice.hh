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
 *       Victor Arribas Raigadas <v.arribas.urjc@gmai.com>
 */

#ifndef QUADROTORICE_H
#define QUADROTORICE_H


#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <quadrotor/interfaces/pose3di.h>
#include <quadrotor/interfaces/navdatai.h>

#include <quadrotor/quadrotorsensors.hh>

namespace quadrotor{

class QuadrotorIce
{
public:
    QuadrotorIce(Ice::CommunicatorPtr ic, const QuadRotorSensors *sensors);
    virtual ~QuadrotorIce();

    void run();
    void start();

private:
    Ice::CommunicatorPtr ic;
    Ice::PropertiesPtr prop;
    Ice::ObjectAdapterPtr adapter;
    boost::thread *ice_thread;

private:
    const QuadRotorSensors *sensor;

};

typedef boost::shared_ptr<QuadrotorIce> QuadrotorIcePtr;

}//NS

#endif // QUADROTORICE_H
