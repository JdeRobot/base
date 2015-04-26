#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <pthread.h>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <referee.h>

#ifndef REFEREE_H
#define REFEREE_H

namespace gazebo {

void* thread_RefereeICE ( void* v );

class Arbitro : public WorldPlugin {
public:

    Arbitro ();
    ~Arbitro();
    virtual void Load (physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    virtual void Init ();
    double GetDistance() const;
    std::string cfgfile_referee_;
private:
    virtual void OnUpdate ();
    void CalculateDistance();

    event::ConnectionPtr updateConnection;
    physics::WorldPtr world_;
    std::string mouse_name_;
    std::string cat_name_;
    physics::ModelPtr mouse_;
    physics::ModelPtr cat_;

    double distance_;

    mutable pthread_mutex_t distance_mtx_;
};

} //gazebo

#endif // REFEREE_H
