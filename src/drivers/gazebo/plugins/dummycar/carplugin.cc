#include <boost/bind.hpp>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

namespace gazebo
{
  	class ModelPush : public ModelPlugin {

		private: ignition::math::Pose3d pose;

  		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr ) {
      			this->model = _parent;
	  		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          		boost::bind(&ModelPush::OnUpdate, this, _1));
			std::cout << "Loading carplugin dummy" << std::endl;
		}

    		public: void OnUpdate(const common::UpdateInfo & ) {
			pose = this->model->WorldPose();
		
  			
  			if (pose.Pos()[1] < 0){
  			    this->model->SetLinearVel(ignition::math::Vector3d(5, 0, 0));
      			if ( pose.Pos()[0] >=50 ) {
			        pose.Pos()[0] = -50;
			        this->model->SetWorldPose(pose);
			    }
			}
			
			if (pose.Pos()[1] >= 0){
			    this->model->SetLinearVel(ignition::math::Vector3d(-5, 0, 0));
      			if ( pose.Pos()[0] <= -50 ) {
			        pose.Pos()[0] = 50;
			        this->model->SetWorldPose(pose);
			    }
			}	

		}
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
  	};
  	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
