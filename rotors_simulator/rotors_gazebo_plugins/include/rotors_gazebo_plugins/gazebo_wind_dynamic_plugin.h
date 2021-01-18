/*
 * Modification of original gazebo_wind_plugin
 */


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "rotors_gazebo_plugins/common.h"

#include "WindSpeed.pb.h"             // Wind speed message
#include "WrenchStamped.pb.h"         // Wind force message

namespace gazebo {
// Default values
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultWindSpeedPubTopic = "wind_speed";
static constexpr double kDefaultWindSpeedMean = 0.0;
static constexpr double kDefaultWindMeanDirection = 0.0;
static constexpr double kDefaultWindMeanDirectionz = 0.0;
static constexpr double kDefaultWindSTDSpeed = 0.0;
static constexpr double kDefaultWindSTDDirection = 0.0;
static constexpr double kDefaultWindSTDDirectionz = 0.0;


/// \brief    This gazebo plugin simulates wind acting on a model.
/// \details  This plugin publishes on a Gazebo topic and instructs the ROS interface plugin to
///           forward the message onto ROS.
class GazeboWindPlugin : public ModelPlugin {
 public:
  GazeboWindPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_force_pub_topic_(mav_msgs::default_topics::EXTERNAL_FORCE),
        wind_speed_pub_topic_(mav_msgs::default_topics::WIND_SPEED),
        wind_speed_mean_(kDefaultWindSpeedMean),
        wind_mean_direction_(kDefaultWindMeanDirection),
        wind_mean_direction_z_(kDefaultWindMeanDirectionz),
        wind_std_speed_(kDefaultWindSTDSpeed),
        wind_std_direction_(kDefaultWindSTDDirection),
        wind_std_direction_z_(kDefaultWindSTDDirectionz),
        wind_gust_mean_(5),
        wind_gust_length_(10),
        wind_gust_length_std_(0.1),
        wind_gust_downtime_(10),
        wind_gust_downtime_std_(0.1),
        wind_gust_decay_(0.5),
        wind_turbulence_mean_(1),
        wind_turbulence_std_(0.1),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(nullptr),
        pubs_and_subs_created_(false) {}

  virtual ~GazeboWindPlugin();

 protected:

  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr update_connection_;
  event::ConnectionPtr update_wind_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_force_pub_topic_;
  std::string wind_speed_pub_topic_;

  // double wind_force_mean_;
  double wind_speed_mean_;
  double wind_mean_direction_;
  double wind_mean_direction_z_;
  double wind_std_speed_;
  double wind_std_direction_;
  double wind_std_direction_z_;
  double wind_gust_mean_;
  double wind_gust_std_;
  double wind_gust_length_;
  double wind_gust_length_std_;
  double wind_gust_downtime_;
  double wind_gust_downtime_std_;
  double wind_gust_decay_;
  double wind_turbulence_mean_;
  double wind_turbulence_std_;

  double windNormSpeed_;
  double windNormDirection_;
  double windNormDirection_z_;
  double windNormNorth_;
  double windNormEast_;
  double windNormDown_;
  double windGustNorth_;
  double windGustEast_;
  double windGustDown_;
  double windTurbNorth_;
  double windTurbEast_;
  double windTurbDown_;

  double v_gust;

  double h;
  double V;
  double L_w;
  double L_u;
  double L_v;
  double omega_w;
  double omega_u;
  double omega_v;
  double start_wait;
  double start_gust;
  double wait_length;
  double gust_elapsed;
  double gust_length;
  bool gust_begin;
  bool gust_ending;
  double v_m;

  double windNorth_;
  double windEast_;
  double windDown_;

  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d wind_direction_;

  float min_x_;
  float min_y_;
  int n_x_;
  int n_y_;
  float res_x_;
  float res_y_;
  std::vector<float> vertical_spacing_factors_;
  std::vector<float> bottom_z_;
  std::vector<float> top_z_;
  std::vector<float> u_;
  std::vector<float> v_;
  std::vector<float> w_;


  // \brief variables for update wind field frequency
  int wind_field_update_frequency;
  int current_update;
  
  /// \brief  Reads wind data from a text file and saves it.
  /// \param[in] custom_wind_field_path Path to the wind field from ~/.ros.
  void ReadCustomWindField(std::string& custom_wind_field_path);
  
  /// \brief Creates normal distribution sample from mean and standard deviation
  double NormalGenerator(double mean, double stdDev);

  /// \brief  Functions for trilinear interpolation of wind field at aircraft position.
  
  /// \brief  Linear interpolation
  /// \param[in]  position y-coordinate of the target point.
  ///             values Pointer to an array of size 2 containing the wind values
  ///                    of the two points to interpolate from (12 and 13).
  ///             points Pointer to an array of size 2 containing the y-coordinate 
  ///                    of the two points to interpolate from.
  ignition::math::Vector3d LinearInterpolation(double position, ignition::math::Vector3d * values, double* points) const;
  
  /// \brief  Bilinear interpolation
  /// \param[in]  position Pointer to an array of size 2 containing the x- and 
  ///                      y-coordinates of the target point.
  ///             values Pointer to an array of size 4 containing the wind values 
  ///                    of the four points to interpolate from (8, 9, 10 and 11).
  ///             points Pointer to an array of size 14 containing the z-coordinate
  ///                    of the eight points to interpolate from, the x-coordinate 
  ///                    of the four intermediate points (8, 9, 10 and 11), and the 
  ///                    y-coordinate of the last two intermediate points (12 and 13).
  ignition::math::Vector3d BilinearInterpolation(double* position, ignition::math::Vector3d * values, double* points) const;
  
  /// \brief  Trilinear interpolation
  /// \param[in]  link_position Vector3 containing the x, y and z-coordinates
  ///                           of the target point.
  ///             values Pointer to an array of size 8 containing the wind values of the 
  ///                    eight points to interpolate from (0, 1, 2, 3, 4, 5, 6 and 7).
  ///             points Pointer to an array of size 14 containing the z-coordinate          
  ///                    of the eight points to interpolate from, the x-coordinate 
  ///                    of the four intermediate points (8, 9, 10 and 11), and the 
  ///                    y-coordinate of the last two intermediate points (12 and 13).
  ignition::math::Vector3d TrilinearInterpolation(ignition::math::Vector3d link_position, ignition::math::Vector3d * values, double* points) const;
  
  gazebo::transport::PublisherPtr wind_force_pub_;
  gazebo::transport::PublisherPtr wind_speed_pub_;

  gazebo::transport::NodePtr node_handle_;

  /// \brief    Gazebo message for sending wind data.
  /// \details  This is defined at the class scope so that it is re-created
  ///           everytime a wind message needs to be sent, increasing performance.
  gz_geometry_msgs::WrenchStamped wrench_stamped_msg_;

  /// \brief    Gazebo message for sending wind speed data.
  /// \details  This is defined at the class scope so that it is re-created
  ///           everytime a wind speed message needs to be sent, increasing performance.
  gz_mav_msgs::WindSpeed wind_speed_msg_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
