#include "referee.hh"

#define cycle 250 //miliseconds, 4 Hz

namespace gazebo {

GZ_REGISTER_WORLD_PLUGIN(Arbitro);

//Global
math::Pose pose;
pthread_mutex_t pose_mutex;

Arbitro::Arbitro() {
    pthread_mutex_init(&distance_mtx_, NULL);
}

Arbitro::~Arbitro() {}

void Arbitro::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    this->world_ = _parent;

    if(!_sdf->HasElement("RefereeCfgFile"))
        this->cfgfile_referee_ = "--Ice.Config=referee.cfg";
    else
        this->cfgfile_referee_ = "--Ice.Config=" + _sdf->GetElement("RefereeCfgFile")->GetValue()->GetAsString();

    if(!_sdf->HasElement("MouseName"))
        this->mouse_name_ = "Redd";
    else
        this->mouse_name_ = _sdf->GetElement("MouseName")->GetValue()->GetAsString();
    if(!_sdf->HasElement("CatName"))
        this->cat_name_ = "Blacky";
    else
        this->cat_name_ = _sdf->GetElement("CatName")->GetValue()->GetAsString();

    mouse_ = world_->GetModel(this->mouse_name_);
    cat_ = world_->GetModel(this->cat_name_);

    pthread_t thr_ice;
    pthread_create(&thr_ice, NULL, &thread_RefereeICE, (void*) this);
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Arbitro::OnUpdate, this));
}

void Arbitro::Init() {
    CalculateDistance();
}

void Arbitro::OnUpdate() {
    struct timeval a, b;
    long diff;
    long totalb, totala;

    while(true) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        CalculateDistance();

        //Sleep Algorithm
        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;
        if (diff < 0 || diff > cycle)
            diff = cycle;
        else
            diff = cycle - diff;

        /*Sleep Algorithm*/
        usleep(diff * 1000);
        if (diff < 33)
            usleep(33 * 1000);
        //printf("CONTROL %.15lf seconds elapsed\n", diff);
    }
}

void Arbitro::CalculateDistance() {
    math::Pose mouse_pose = mouse_->GetWorldPose();
    math::Pose cat_pose = cat_->GetWorldPose();
    math::Vector3 distance_vector = mouse_pose.pos - cat_pose.pos;
    pthread_mutex_lock(&distance_mtx_);
    this->distance_ = distance_vector.GetLength();
    pthread_mutex_unlock(&distance_mtx_);
}

double Arbitro::GetDistance() const {
    pthread_mutex_lock(&distance_mtx_);
    double temp = this->distance_;
    pthread_mutex_unlock(&distance_mtx_);
    return temp;
}

class RefereeI : virtual public jderobot::Referee {
public:
    RefereeI (Arbitro* r) {
        this->r = r;
    }

    virtual ~RefereeI () {}

    virtual double getDistance ( const Ice::Current& ) {
        return r->GetDistance();
    }

private:
    Arbitro* r;
};

void* thread_RefereeICE ( void* v ) {
    Arbitro* referee = (Arbitro*)v;
    char* name = (char*) referee->cfgfile_referee_.c_str();
    Ice::CommunicatorPtr ic;
    int argc = 1;
    Ice::PropertiesPtr prop;
    char* argv[] = {name};

    try {
        ic = Ice::initialize(argc, argv);
        prop = ic->getProperties();

        std::string ref_name = prop->getProperty("Referee.Name");
        std::string endpoints = prop->getProperty("Referee.Endpoints");
        std::cout << "Referee Endpoints > " << endpoints << std::endl;
        Ice::ObjectAdapterPtr Adapter =
                ic->createObjectAdapterWithEndpoints("AdapterReferee", endpoints);
        Ice::ObjectPtr refereeI = new RefereeI(referee);
        Adapter->add(refereeI, ic->stringToIdentity(ref_name));
        Adapter->activate();

        ic->waitForShutdown();
    } catch (const Ice::Exception& e) {
        std::cerr << e << std::endl;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
    }
    if (ic) {
        try {
            ic->destroy();
        } catch (const Ice::Exception& e) {
            std::cerr << e << std::endl;
        }
    }
}

} //gazebo
