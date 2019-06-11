#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

using namespace ignition;

namespace gazebo
{
	class Wall3 : public ModelPlugin {

		private: math::Pose3d pose;
		private: bool flag;
		private: double vel1; 
		private: double vel2; 

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr ) {
			this->model = _parent;
			flag = true;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Wall3::OnUpdate, this, _1));
			std::cout << "Loading wall 3" << std::endl;
			this->vel1 = 0.5 + double(rand())/RAND_MAX*1.5;
			this->vel2 = (0.5 + double(rand())/RAND_MAX*1.5)*-1;
			
		}

		public: void OnUpdate(const common::UpdateInfo & ) {
			pose = this->model->WorldPose();

			if  (flag) {
				this->model->SetLinearVel(math::Vector3d(this->vel1, 0, 0));
			}
			if ( pose.Pos().X() >=4 ) {
				pose.Pos().X() = 4;
				this->model->SetWorldPose(pose);
				flag = false;
			}
			if (!flag) {
				this->model->SetLinearVel(math::Vector3d(this->vel2, 0, 0));
			}	
			if ( pose.Pos().X() <= 1.25 ) {
				pose.Pos().X() = 1.25;
				this->model->SetWorldPose(pose);
				flag = true;
			}

		}
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
	};
	GZ_REGISTER_MODEL_PLUGIN(Wall3)
}
