#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  this->foot_contact_force_pub = this->nh_->advertise<geometry_msgs::WrenchStamped>("/lite3/"+_sensor->Name()+"/contact_force", 100);

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  foot_contact_force.header.stamp=ros::Time::now();
  if(contacts.contact_size()>=1){
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    // std::cout << "Collision between[" << contacts.contact(i).collision1()
     //         << "] and [" << contacts.contact(i).collision2() << "]\n"; 

    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      /* std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n"; */
   resultant_force_z+=contacts.contact(i).wrench(j).body_1_wrench().force().z();
    }
  }
   foot_contact_force.wrench.force.z=resultant_force_z/contacts.contact_size();
  }
  else{
   foot_contact_force.wrench.force.z=0.0;
  }
  this->foot_contact_force_pub.publish(foot_contact_force);
  resultant_force_z=0.0;


}
/* This tutorial demonstrates the process of creating a contact sensor and getting the contact data via a plugin or a message. 
A contact sensor detects collisions between two objects and reports the location of the contact associated forces.
https://classic.gazebosim.org/tutorials?tut=contact_sensor&cat=sensors
 */
