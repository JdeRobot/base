#include <boost/bind.hpp>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

using namespace ignition;

namespace gazebo
{
  	class ModelPush : public ModelPlugin {

		private: math::Pose3d pose;

  		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr ) {
      		
			this->model = _parent;
			range = (max_val - min_val) + 1;
	  		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          		boost::bind(&ModelPush::OnUpdate, this, _1));
			std::cout << "Loading CrAzY ball" << std::endl;
		}

    	public: void OnUpdate(const common::UpdateInfo & ) {
			pose = this->model->WorldPose();

			srand(time(NULL));
			vX = (rand() % range + min_val)/10.0;
			vY = (rand() % range + min_val)/10.0;
			this->model->SetLinearVel(math::Vector3d(vX, vY, 0));


		}
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: float vX, vY;
		private: int min_val = -5;
		private: int max_val = 5;
		private: int range = 0;
  	};
  	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
