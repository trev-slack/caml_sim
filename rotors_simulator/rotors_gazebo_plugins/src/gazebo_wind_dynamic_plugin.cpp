/*
 * Modification of original gazebo_wind_plugin
 * Trevor Slack, trevor.slack@colorado.edu
 */

#include "rotors_gazebo_plugins/gazebo_wind_dynamic_plugin.h"
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <math.h>
#include <string>
#include <random>

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  
}

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
    ROS_DEBUG("Hello %s", "World");
  }

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

  // Create Gazebo Node.
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d >();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windForcePubTopic", wind_force_pub_topic_,
                           wind_force_pub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_,
                           wind_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);

  // Get the wind speed params from SDF.

  // wind mean
  // wind speed mean speed
  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_,
                      wind_speed_mean_);
  // mean direction
  getSdfParam<double>(_sdf, "windMeanDirection", wind_mean_direction_, wind_mean_direction_);
  getSdfParam<double>(_sdf, "windMeanDirectionz", wind_mean_direction_z_, wind_mean_direction_z_);
  // wind standard deviation
  // mean wind field standard deviation of speed
  getSdfParam<double>(_sdf, "windSTDSpeed", wind_std_speed_, wind_std_speed_);
  // mean wind field standard deviation of direction
  getSdfParam<double>(_sdf, "windSTDDirection", wind_std_direction_, wind_std_direction_);
  getSdfParam<double>(_sdf, "windSTDDirectionz", wind_std_direction_z_, wind_std_direction_z_);
  // gust bounds
  // uniform gust bounds on speed uncertainty
  getSdfParam<double>(_sdf, "windGustSpeed", wind_gust_speed_, wind_gust_speed_);
  // uniform gust bounds on direction uncertainty
  getSdfParam<double>(_sdf, "windGustDirection", wind_gust_direction_, wind_gust_direction_);
  getSdfParam<double>(_sdf, "windGustDirectionz", wind_gust_direction_z_, wind_gust_direction_z_);
  // turbulence bounds
  // turbulence uniform gust bounds on speed uncertainty
  getSdfParam<double>(_sdf, "turbulanceGustSpeed", turbulance_gust_speed_, turbulance_gust_speed_);
  // turbulence uniform gust bounds on direction uncertainty
  getSdfParam<double>(_sdf, "turbulanceGustDirection", turbulance_gust_direction_, turbulance_gust_direction_);
  getSdfParam<double>(_sdf, "turbulanceGustDirectionz", turbulance_gust_direction_z_, turbulance_gust_direction_z_);

  std::string custom_wind_field_path;
  getSdfParam<std::string>(_sdf, "customWindFieldPath", custom_wind_field_path,
                      custom_wind_field_path);
  ReadCustomWindField(custom_wind_field_path);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWindPlugin::OnUpdate, this,_1));

}


// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current simulation time.
  common::Time now = world_->SimTime();
  
  ignition::math::Vector3d wind_velocity(0.0, 0.0, 0.0);

  // get current aircraft pose
  const ignition::math::v4::Quaternion<double> link_orientation = link_->WorldPose().Rot();
  const ignition::math::Vector3d link_position = link_->WorldPose().Pos();
  // scaling factor for hieght
  float turbulance_speed_scale_factor = 1;

  // Calculate the x, y index of the grid points with x, y-coordinate 
  // just smaller than or equal to aircraft x, y position.
  std::size_t x_inf = floor((link_position.X() - min_x_) / res_x_);
  std::size_t y_inf = floor((link_position.Y() - min_y_) / res_y_);

  // In case aircraft is on one of the boundary surfaces at max_x or max_y,
  // decrease x_inf, y_inf by one to have x_sup, y_sup on max_x, max_y.
  if (x_inf == n_x_ - 1u) {
    x_inf = n_x_ - 2u;
  }
  if (y_inf == n_y_ - 1u) {
    y_inf = n_y_ - 2u;
  }
  //gzdbg << "[gazebo_wind_plugin] x_inf " << x_inf << " y_inf " << y_inf << ".\n";
  float map_height = bottom_z_[x_inf + y_inf*n_x_];
  //gzdbg << "[gazebo_wind_plugin] " << map_height << "\n";

  //dryden turbulance model



  tf::Quaternion q(link_orientation.X(), link_orientation.Y(), link_orientation.Z(), link_orientation.W());
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);



  // Calculate the wind speed using method developed by Katherine Glasheen 1/23/2020
  // sample nominal wind from the distributions specified
  windNormSpeed_ = NormalGenerator(wind_speed_mean_,wind_std_speed_);
  windNormDirection_ = NormalGenerator(wind_mean_direction_,wind_std_direction_);
  windNormDirection_z_ = NormalGenerator(wind_mean_direction_z_,wind_std_direction_);
  // wind norm
  windNormNorth_ = windNormSpeed_ * cos(windNormDirection_+yaw);
  windNormEast_ = windNormSpeed_ * sin(windNormDirection_+yaw);
  windNormDown_ = windNormSpeed_ * sin(windNormDirection_z_+pitch);

  // create 0 to 1 random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1);//uniform distribution between 0 and 1

  // sample wind gust
  windGustSpeed_ = wind_gust_speed_ * dis(gen) * signbit(dis(gen)-0.5);
  windGustDirection_ = wind_gust_direction_ * dis(gen) * signbit(dis(gen)-0.5);
  windGustDirection_z_ = wind_gust_direction_z_ * dis(gen) * signbit(dis(gen)-0.5);
  // wind gust
  windGustNorth_ = windGustSpeed_ * cos(windGustDirection_+link_orientation.Z());
  windGustEast_ = windGustSpeed_ * sin(windGustDirection_+link_orientation.Z());
  windGustDown_ = windGustSpeed_ * sin(windGustDirection_z_+link_orientation.Y());

  // sample wind turbulance
  windTurbulanceSpeed_ = turbulance_gust_speed_ * dis(gen) * signbit(dis(gen)-0.5);
  // modify turbulance by height
  windTurbulanceSpeed_= windTurbulanceSpeed_ * (turbulance_speed_scale_factor/(link_position.Z()));
  windTurbulanceDirection_ = turbulance_gust_direction_ * dis(gen) * signbit(dis(gen)-0.5);
  windTurbulanceDirection_z_ = turbulance_gust_direction_z_ * dis(gen) * signbit(dis(gen)-0.5);
  //wind turbulance
  windTurbulanceNorth_ = windTurbulanceSpeed_ * cos(windTurbulanceDirection_+link_orientation.Z());
  windTurbulanceEast_ = windTurbulanceSpeed_ * sin(windTurbulanceDirection_+link_orientation.Z());
  windTurbulanceDown_ = windTurbulanceSpeed_ * sin(windTurbulanceDirection_z_+link_orientation.Y());

  // calculate wind velocity
  wind_velocity.X() = windNormNorth_ + windGustNorth_ + windTurbulanceNorth_;
  wind_velocity.Y() = windNormEast_ + windGustEast_ + windTurbulanceEast_;
  wind_velocity.Z() = windNormDown_ + windGustDown_ + windTurbulanceDown_;
  
  wind_speed_msg_.mutable_header()->set_frame_id(frame_id_);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wind_speed_msg_.mutable_velocity()->set_x(wind_velocity.X());
  wind_speed_msg_.mutable_velocity()->set_y(wind_velocity.Y());
  wind_speed_msg_.mutable_velocity()->set_z(wind_velocity.Z());

  wind_speed_pub_->Publish(wind_speed_msg_);
}

void GazeboWindPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message.
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // ========= WRENCH STAMPED MSG SETUP ========= //
  // ============================================ //
  wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
      "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========== WIND SPEED MSG SETUP ============ //
  // ============================================ //
  wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
      "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

void GazeboWindPlugin::ReadCustomWindField(std::string& custom_wind_field_path) {
  std::ifstream fin;
  fin.open(custom_wind_field_path);
  if (fin.is_open()) {
    std::string data_name;
    float data;
    // Read the line with the variable name.
    while (fin >> data_name) {
      // Save data on following line into the correct variable.
      if (data_name == "min_x:") {
        fin >> min_x_;
      } else if (data_name == "min_y:") {
        fin >> min_y_;
      } else if (data_name == "n_x:") {
        fin >> n_x_;
      } else if (data_name == "n_y:") {
        fin >> n_y_;
      } else if (data_name == "res_x:") {
        fin >> res_x_;
      } else if (data_name == "res_y:") {
        fin >> res_y_;
      // } else if (data_name == "vertical_spacing_factors:") {
      //   while (fin >> data) {
      //     vertical_spacing_factors_.push_back(data);
      //     if (fin.peek() == '\n') break;
      //   }
      } else if (data_name == "bottom_z:") {
        while (fin >> data) {
          bottom_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } //else if (data_name == "top_z:") {
      //   while (fin >> data) {
      //     top_z_.push_back(data);
      //     if (fin.peek() == '\n') break;
      //   }
      // } else if (data_name == "u:") {
      //   while (fin >> data) {
      //     u_.push_back(data);
      //     if (fin.peek() == '\n') break;
      //   }
      // } else if (data_name == "v:") {
      //   while (fin >> data) {
      //     v_.push_back(data);
      //     if (fin.peek() == '\n') break;
      //   }
      // } else if (data_name == "w:") {
      //   while (fin >> data) {
      //     w_.push_back(data);
      //     if (fin.peek() == '\n') break;
      //   }
      else {
        // If invalid data name, read the rest of the invalid line, 
        // publish a message and ignore data on next line. Then resume reading.
        std::string restOfLine;
        getline(fin, restOfLine);
        //gzerr << " [gazebo_wind_plugin] Invalid data name '" << data_name << restOfLine <<
              "' in custom wind field text file. Ignoring data on next line.\n";
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
    fin.close();
    gzdbg << "[gazebo_wind_plugin] Successfully read custom wind field from text file.\n";
  } else {
    gzerr << "[gazebo_wind_plugin] Could not open custom wind field text file.\n";
  }
}


// Generates a normal distribution
double GazeboWindPlugin::NormalGenerator(double mean, double stdDev) {
    static std::default_random_engine generator(time(0)); // intialize generator
    std::normal_distribution<double> distribution(mean,stdDev); // distribution
    return distribution(generator); // distribution operator() always take the same generator with the same state
}


ignition::math::Vector3d GazeboWindPlugin::LinearInterpolation(
  double position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d value = values[0] + (values[1] - values[0]) /
                        (points[1] - points[0]) * (position - points[0]);
  return value;
}

ignition::math::Vector3d GazeboWindPlugin::BilinearInterpolation(
  double* position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d intermediate_values[2] = { LinearInterpolation(
                                             position[0], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[0], &(values[2]), &(points[2])) };
  ignition::math::Vector3d value = LinearInterpolation(
                          position[1], intermediate_values, &(points[4]));
  return value;
}

ignition::math::Vector3d GazeboWindPlugin::TrilinearInterpolation(
  ignition::math::Vector3d link_position, ignition::math::Vector3d * values, double* points) const {
  double position[3] = {link_position.X(),link_position.Y(),link_position.Z()};
  ignition::math::Vector3d intermediate_values[4] = { LinearInterpolation(
                                             position[2], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[2], &(values[2]), &(points[2])),
                                           LinearInterpolation(
                                             position[2], &(values[4]), &(points[4])),
                                           LinearInterpolation(
                                             position[2], &(values[6]), &(points[6])) };
  ignition::math::Vector3d value = BilinearInterpolation(
    &(position[0]), intermediate_values, &(points[8]));
  return value;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);

}  // namespace gazebo
