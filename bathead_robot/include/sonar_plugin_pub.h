/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 */

#ifndef BJARKI_GAZEBO_ROS_SONAR_H
#define BJARKI_GAZEBO_ROS_SONAR_H

#include <ros/ros.h>

#include <gazebo/plugins/SonarPlugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <sensor_msgs/Range.h>

namespace gazebo
{

   class GazeboRosSonar : public SensorPlugin
   {
      /// \brief Constructor
      public: GazeboRosSonar();

      /// \brief Destructor
      public: virtual ~GazeboRosSonar();

      /// \brief Load the controller
      protected: virtual void Load( sensors::SensorPtr _sensor, sdf::ElementPtr _sdf );

      /// Reset the controller
      protected: virtual void Reset();

      /// \brief Update the controller
      protected: virtual void Update();

      private:
        ros::NodeHandle* node_handle_;
        ros::Publisher publisher_;

        sensor_msgs::Range range_;

        std::string namespace_;
        std::string topic_;
        std::string frame_id_;

   };

}

#endif