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
  getSdfParam<double>(_sdf, "windMeanDirectionNorthEast", wind_mean_direction_, wind_mean_direction_);
  getSdfParam<double>(_sdf, "windMeanDirectionDown", wind_mean_direction_z_, wind_mean_direction_z_);
  // wind standard deviation
  // mean wind field standard deviation of speed
  getSdfParam<double>(_sdf, "windSTDSpeed", wind_std_speed_, wind_std_speed_);
  // mean wind field standard deviation of direction
  getSdfParam<double>(_sdf, "windSTDDirectionNorthEast", wind_std_direction_, wind_std_direction_);
  getSdfParam<double>(_sdf, "windSTDDirectionDown", wind_std_direction_z_, wind_std_direction_z_);

  // gust params
  getSdfParam<double>(_sdf, "windGustMean", wind_gust_mean_,
                      wind_gust_mean_);
  getSdfParam<double>(_sdf, "windGustSTD", wind_gust_std_,
                      wind_gust_std_);
  getSdfParam<double>(_sdf, "windGustLength", wind_gust_length_,
                      wind_gust_length_);
  getSdfParam<double>(_sdf, "windGustLengthSTD", wind_gust_length_std_,
                      wind_gust_length_std_);
  getSdfParam<double>(_sdf, "windGustDownTime", wind_gust_downtime_,
                      wind_gust_downtime_);
  getSdfParam<double>(_sdf, "windGustDownTimeSTD", wind_gust_downtime_std_,
                      wind_gust_downtime_std_);
  getSdfParam<double>(_sdf, "windGustDecay", wind_gust_decay_,
                      wind_gust_decay_);
  //turbulance params
  getSdfParam<double>(_sdf, "windTurbulenceMean", wind_turbulence_mean_,
                      wind_turbulence_mean_);
  getSdfParam<double>(_sdf, "windTurbulenceSTD", wind_turbulence_std_,
                      wind_turbulence_std_);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");
  // gust initial conditions
  wait_length = NormalGenerator(wind_gust_downtime_,wind_gust_downtime_std_);
  gust_length = NormalGenerator(wind_gust_length_,wind_gust_length_std_);
  max_gust_time = NormalGenerator(gust_length/2,wind_gust_length_std_);
  // max gust
  v_m = NormalGenerator(wind_gust_mean_,wind_gust_std_);
  start_wait = world_->SimTime().sec;
  gust_begin = true;
  gust_ending = true;

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
  const ignition::math::v6::Quaternion<double> link_orientation = link_->WorldPose().Rot();
  const ignition::math::Vector3d link_position = link_->WorldPose().Pos();
  ignition::math::Vector3d current_vel_vec = link_->RelativeLinearVel();
  double current_vel = sqrt(pow(current_vel_vec.X(),2)+pow(current_vel_vec.Y(),2)+pow(current_vel_vec.Z(),2));

  // nominal wind
  // sample nominal wind from the distributions specified
  windNormSpeed_ = NormalGenerator(wind_speed_mean_,wind_std_speed_);
  windNormDirection_ = NormalGenerator(wind_mean_direction_,wind_std_direction_);
  windNormDirection_z_ = NormalGenerator(wind_mean_direction_z_,wind_std_direction_);
  // wind norm
  windNormNorth_ = windNormSpeed_ * cos(windNormDirection_);
  windNormEast_ = windNormSpeed_ * sin(windNormDirection_);
  windNormDown_ = windNormSpeed_ * sin(windNormDirection_z_);

  // dryden turbulance model (specification MIL-F-8785C, low altitude < 1000ft)
  // altitude 
  h = link_position.Z();
  if (h==0) {
    h = 0.0001;
  }
  //gzdbg << "altitude: " << h << std::endl;
  // velocity
  V = current_vel;
  if (V==0) {
    V = 0.0001;
  }
  //gzdbg << "velocity: " << V << std::endl;
  // turbulance scale length
  L_w = h;
  L_u = h/(pow((0.177+0.000823*h),1.2));
  L_v = L_u;
  //gzdbg << "u turbulance scale: " << L_u << std::endl;
  // turbulance intensity
  omega_w = 0.1*windNormSpeed_;
  omega_u = omega_w/(pow((0.177+0.000823*h),0.4));
  omega_v = omega_u;
  //gzdbg << "u turbulance intensity: " << omega_u << std::endl;

  // gaussian noise (place holder for band-limited)
  std::default_random_engine generator;
  std::normal_distribution<double> dist(wind_turbulence_mean_, wind_turbulence_std_);
  double s1 = dist(generator);
  double s2 = dist(generator);
  double s3 = dist(generator);
  //gzdbg << s1 << " " << s2 << " " << s3 << "\n";
  // dryden transfer functions
  windTurbNorth_ = omega_u*sqrt((2*L_u)/(3.14159*V))*(1/(1+L_u/V*s1));
  windTurbEast_ = omega_v*sqrt((2*L_v)/(3.14159*V))*((1+(3.46410*L_v)/V)/(pow((1+2*L_v/V*s2),2)));
  windTurbDown_ = omega_w*sqrt((2*L_w)/(3.14159*V))*((1+(3.46410*L_w)/V)/(pow((1+2*L_w/V*s3),2)));
  //gzdbg << "turbulance: " << windTurbNorth_ << " " << windTurbEast_ << " " << windTurbDown_ << std::endl;
  //gzdbg << "turbulance: " << windTurbNorth_ << ", " << windTurbEast_ << ", " << windTurbDown_ << std::endl;


  // 1-cosine gut model
  // gusting
  if (now.sec-start_wait > wait_length) {
    if (gust_begin == true) {
      start_gust = now.sec;
      gust_begin = false;
    }
    gust_elapsed = now.sec-start_gust;
    // gust gain
    if (gust_elapsed < max_gust_time) {
      v_gust = v_m/2*(1-cos((3.14159*gust_elapsed)/max_gust_time)); 
      //gzdbg << "Gust Gain: " << v_gust << std::endl;
    }
    // gust decay
    else if (gust_elapsed > max_gust_time && gust_elapsed < gust_length) {
      if (gust_ending==true) {
        decay_start = gust_elapsed;
        gust_ending = false;
      }
      v_gust = v_m/2*(1-cos((3.14159*(gust_elapsed-decay_start))/(gust_length-max_gust_time)+3.14159)); 
      //gzdbg << "Gust Decay: " << v_gust << std::endl;
    }
    //gust ended
    else {
      v_gust = 0;
      // new gust conditions
      gust_length = NormalGenerator(wind_gust_length_,wind_gust_length_std_);
      wait_length = NormalGenerator(wind_gust_downtime_,wind_gust_downtime_std_);
      v_m = NormalGenerator(wind_gust_mean_,wind_gust_std_);
      // time of peak gust
      max_gust_time = NormalGenerator(gust_length/2,wind_gust_length_std_);
      start_wait = now.sec;
      gust_begin = true;  
      gust_ending = true; 
    }
  } 
  // not gusting
  else {
    v_gust = 0;
    //gzdbg << "Not gusting" << std::endl;
  }




  // apply gust aligned with nominal wind
  windGustNorth_ = v_gust * cos(windNormDirection_);
  windGustEast_ = v_gust * sin(windNormDirection_);
  windGustDown_ = v_gust * sin(windNormDirection_z_);

  // sum wind components
  //gzdbg << "Norm: " << windNormNorth_ << "\nTurb: " << windTurbNorth_ << "\nGust: " << windGustNorth_ << "\n\n";
  windNorth_ = windNormNorth_ + windTurbNorth_ + windGustNorth_;
  windEast_ = windNormEast_ + windTurbEast_ + windGustEast_;
  windDown_ = windNormDown_ + windTurbDown_ + windGustDown_;
  ignition::math::Vector3 wind_velocity_world(windNorth_, windEast_, windDown_);

  // transform to body frame
  wind_velocity = link_orientation.RotateVector(wind_velocity_world);

  // create message
  wind_speed_msg_.mutable_header()->set_frame_id(frame_id_);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  wind_speed_msg_.mutable_velocity()->set_x(wind_velocity.X());
  wind_speed_msg_.mutable_velocity()->set_y(wind_velocity.Y());
  wind_speed_msg_.mutable_velocity()->set_z(wind_velocity.Z());
  // publish
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
