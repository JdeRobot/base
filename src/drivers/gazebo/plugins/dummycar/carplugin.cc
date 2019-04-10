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
	  		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          		boost::bind(&ModelPush::OnUpdate, this, _1));
			std::cout << "Loading carplugin dummy" << std::endl;
		}

    		public: void OnUpdate(const common::UpdateInfo & ) {
			pose = this->model->WorldPose();
		
  			
  			if (pose.Pos().Y() < 0){
  			    this->model->SetLinearVel(math::Vector3d(5, 0, 0));
      			if ( pose.Pos().X() >=50 ) {
			        pose.Pos().X() = -50;
			        this->model->SetWorldPose(pose);
			    }
			}
			
			if (pose.Pos().Y() >= 0){
			    this->model->SetLinearVel(math::Vector3d(-5, 0, 0));
      			if ( pose.Pos().X() <= -50 ) {
			        pose.Pos().X() = 50;
			        this->model->SetWorldPose(pose);
			    }
			}	

		}
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
  	};
  	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
