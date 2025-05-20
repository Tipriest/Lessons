#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
    private:

    ros::NodeHandle* nh_= new ros::NodeHandle("foot_contact");
    double resultant_force_z=0;
    geometry_msgs::WrenchStamped foot_contact_force;
    ros::Publisher foot_contact_force_pub;

  };

  
}
#endif
/* This tutorial demonstrates the process of creating a contact sensor and getting the contact data via a plugin or a message. 
A contact sensor detects collisions between two objects and reports the location of the contact associated forces.
https://classic.gazebosim.org/tutorials?tut=contact_sensor&cat=sensors
 */