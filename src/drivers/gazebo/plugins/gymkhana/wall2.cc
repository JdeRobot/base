#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

using namespace ignition;

namespace gazebo
{
	class Wall2 : public ModelPlugin {

		private: math::Pose3d pose;
		private: bool flag;
		private: double vel1; 
		private: double vel2; 

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr ) {
			this->model = _parent;
			flag = true;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&Wall2::OnUpdate, this, _1));
			std::cout << "Loading wall 2" << std::endl;
			this->vel1 = 0.5 + double(rand())/RAND_MAX*1.5;
			this->vel2 = (0.5 + double(rand())/RAND_MAX*1.5)*-1;
		}

		public: void OnUpdate(const common::UpdateInfo & ) {
			pose = this->model->WorldPose();

			if  (flag) {
				this->model->SetLinearVel(math::Vector3d(0, 0, this->vel1));
			}
			if ( pose.Pos().Z() >= 2.8 ) {
				pose.Pos().Z() = 2.8;
				this->model->SetWorldPose(pose);
				flag = false;
			}
			if (!flag) {
				this->model->SetLinearVel(math::Vector3d(0, 0, this->vel2));
			}	
			if ( pose.Pos().Z() <= 0) {
				pose.Pos().Z() = 0;
				this->model->SetWorldPose(pose);
				flag = true;
			}

		}
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
	};
	GZ_REGISTER_MODEL_PLUGIN(Wall2)
}
