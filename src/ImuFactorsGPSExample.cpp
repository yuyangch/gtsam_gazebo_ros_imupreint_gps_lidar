/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuFactorsExample
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor
 * navigation code.
 * @author Garrett (ghemann@gmail.com), Luca Carlone
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in
 * conjunction with GPS
 *  - imuFactor is used by default. You can test combinedImuFactor by
 *  appending a `-c` flag at the end (see below for example command).
 *  - we read IMU and GPS data from a CSV file, with the following format:
 *  A row starting with "i" is the first initial position formatted with
 *  N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  A row starting with "0" is an imu measurement
 *  (body frame - Forward, Right, Down)
 *  linAccX, linAccY, linAccZ, angVelX, angVelY, angVelX
 *  A row starting with "1" is a gps correction formatted with
 *  N, E, D, qX, qY, qZ, qW
 * Note that for GPS correction, we're only using the position not the
 * rotation. The rotation is provided in the file for ground truth comparison.
 *
 *  See usage: ./ImuFactorsExample --help
 */

#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <cstring>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace po = boost::program_options;


ros::Publisher  pubImuOdometry;
ros::Subscriber subImu;
ros::Subscriber subOdometry;
std::shared_ptr<PreintegrationType> preintegrated;
double lastImuT_imu=-1;


bool imu2;

long int gps_msg_count=0;
int correction_count=0;
NavState prevStateOdom;
imuBias::ConstantBias prevBiasOdom;

/////////////////////GPS///////////////////
nav_msgs::Odometry CurOdom;
/////////////noise model////////////////////

auto pose_noise_model = noiseModel::Diagonal::Sigmas(
    (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
        .finished());  // rad,rad,rad,m, m, m
auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
///////////temporal container graph////////

NonlinearFactorGraph* graph = new NonlinearFactorGraph();
//////////////ISAM2///////////////
ISAM2* isam2 = 0;
bool use_isam=true;
Values initial_values;
//////////////mutex lock //////////////
std::mutex mtx;
/////////////////////nav states////////////////////
NavState prop_state;
po::variables_map parseOptions(int argc, char* argv[]) {
  po::options_description desc;
  desc.add_options()("help,h", "produce help message")(
      "data_csv_path", po::value<string>()->default_value("imuAndGPSdata.csv"),
      "path to the CSV file with the IMU data")(
      "output_filename",
      po::value<string>()->default_value("imuFactorExampleResults.csv"),
      "path to the result file to use")("use_isam", po::bool_switch(),
                                        "use ISAM as the optimizer");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    cout << desc << "\n";
    exit(1);
  }

  return vm;
}

boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  /*
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  */
  double accel_noise_sigma = 0.03924;
  double gyro_noise_sigma = 0.0205689024915;
  double accel_bias_rw_sigma = 0.04905;
  double gyro_bias_rw_sigma = 0.0001454441043;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_int =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  return p;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    std::lock_guard<std::mutex> lock(mtx);

    static tf::TransformBroadcaster tfMap2Odom;
    static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, "map", "odom"));

    float p_x = odomMsg->pose.pose.position.x;
    float p_y = odomMsg->pose.pose.position.y;
    float p_z = odomMsg->pose.pose.position.z;
    float r_x = odomMsg->pose.pose.orientation.x;
    float r_y = odomMsg->pose.pose.orientation.y;
    float r_z = odomMsg->pose.pose.orientation.z;
    float r_w = odomMsg->pose.pose.orientation.w;


    gps_msg_count++;

    if (gps_msg_count%100==0){


      correction_count++;

      // Adding IMU factor and GPS factor and optimizing.
      auto preint_imu =
          dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);


      ImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1),
                           X(correction_count), V(correction_count),
                           B(correction_count - 1), preint_imu);


      graph->add(imu_factor);

      imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));

      graph->add(BetweenFactor<imuBias::ConstantBias>(
          B(correction_count - 1), B(correction_count), zero_bias,
          bias_noise_model));

      auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);

      GPSFactor gps_factor(X(correction_count),
                           Point3(p_x,   // N,
                                  p_y,   // E,
                                  p_z),  // D,
                           correction_noise);
      graph->add(gps_factor);

      // Now optimize and compare results.
      prop_state = preintegrated->predict(prevStateOdom, prevBiasOdom);
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prevBiasOdom);

      Values result;

      if (use_isam) {
        isam2->update(*graph, initial_values);
        isam2->update();
        result = isam2->calculateEstimate();

        // reset the graph
        graph->resize(0);
        initial_values.clear();

      } else {
        LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
        result = optimizer.optimize();
      }

      // Overwrite the beginning of the preintegration for the next step.
      prevStateOdom = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prevBiasOdom = result.at<imuBias::ConstantBias>(B(correction_count));

      /////////////////////reset prev_state//////////



      // Reset the preintegration object.
      preintegrated->resetIntegrationAndSetBias(prevBiasOdom);




      // Print out the position and orientation error for comparison.
      /*
      Vector3 gtsam_position = prev_state.pose().translation();
      Vector3 position_error = gtsam_position - gps.head<3>();
      current_position_error = position_error.norm();

      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
      Quaternion quat_error = gtsam_quat * gps_quat.inverse();
      quat_error.normalize();
      Vector3 euler_angle_error(quat_error.x() * 2, quat_error.y() * 2,
                                quat_error.z() * 2);

      current_orientation_error = euler_angle_error.norm();
      */
    }

}


void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
{



    std::lock_guard<std::mutex> lock(mtx);


        //static tf::TransformBroadcaster tfodom2map;
        //static tf::Transform odom_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        //tfodom2map.sendTransform(tf::StampedTransform(odom_to_map, imu_raw->header.stamp, "odom", "map"));

    //sensor_msgs::Imu thisImu = imuConverter(*imu_raw);
    sensor_msgs::Imu thisImu=*imu_raw;
    //imuQueOpt.push_back(thisImu);
    //imuQueImu.push_back(thisImu);

    //if (doneFirstOpt == false)
    //    return;

    double imuTime = ROS_TIME(&thisImu);
    double dt;
    if (imu2==true)
      dt = (lastImuT_imu < 0) ? (1.0 / 1000.0) : (imuTime - lastImuT_imu);
    else
      dt = (lastImuT_imu < 0) ? (1.0 / 50) : (imuTime - lastImuT_imu);
    lastImuT_imu = imuTime;

    // integrate this single imu message
    
    float gravity_adjustment=0;
    if (imu2==false){gravity_adjustment=9.81;}
    preintegrated->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z-gravity_adjustment),
                                            gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

    // predict odometry
    gtsam::NavState currentState = preintegrated->predict(prevStateOdom, prevBiasOdom);

    // publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "odom_imu";

    // transform imu pose to ldiar
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    gtsam::Pose3 lidarPose = imuPose;

    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
    
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    (pubImuOdometry).publish(odometry);
}




int main(int argc, char* argv[]) {
  string subtopic=argv[1];
  string pubtopic=argv[2];

  cout<<"subtopic is "<<subtopic;
  cout<<"pubtopic is "<<pubtopic;
  if (subtopic.compare("/imu2")==0)
    imu2=true;
  else
    imu2=false;
  //if pubtopic

  imu2=true;
  ////////////////////////ROS stuff/////////////
  ros::init(argc, argv, "roboat_loam");

  ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe(subtopic, 1000, chatterCallback);
  subImu      = n.subscribe<sensor_msgs::Imu>  (subtopic,                  2000, imuHandler,   ros::TransportHints().tcpNoDelay());
  pubImuOdometry = n.advertise<nav_msgs::Odometry> (pubtopic, 2000);
  subOdometry = n.subscribe<nav_msgs::Odometry>("/body_pose_ground_truth", 5,    &odometryHandler, ros::TransportHints().tcpNoDelay());
  //////////////////ROS stuff ends////////

   //////////////initialization//////////////////////////////////////////////////////////////////////
  //string data_filename, output_filename;

  //bool use_isam = true;

  po::variables_map var_map = parseOptions(argc, argv);

  //data_filename = findExampleDataFile(var_map["data_csv_path"].as<string>());
  //output_filename = var_map["output_filename"].as<string>();
  use_isam = var_map["use_isam"].as<bool>();


  if (use_isam) {
    printf("Using ISAM2\n");
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2 = new ISAM2(parameters);

  } else {
    printf("Using Levenberg Marquardt Optimizer\n");
  }

  // Set up output file for plotting errors
  //FILE* fp_out = fopen(output_filename.c_str(), "w+");
  //fprintf(fp_out,
  //        "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,"
  //        "gt_qy,gt_qz,gt_qw\n");

  // Begin parsing the CSV file.  Input the first line for initialization.
  // From there, we'll iterate through the file and we'll preintegrate the IMU
  // or add in the GPS given the input.
  //ifstream file(data_filename.c_str());
  //string value;
  ///////////////////////////////////////////////get initial state //////////////////////////////////////////////
  // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  Vector10 initial_state;

  //getline(file, value, ',');  // i
  for (int i = 0; i < 9; i++) {
    initial_state(i) = 0.0;
  }
  initial_state(6)=1.0;
 // getline(file, value, '\n');
 // initial_state(9) = stof(value.c_str());
  cout << "initial state:\n" << initial_state.transpose() << "\n\n";
   /////////////////////////////////////////////add initial state as priror//////////////////////////////////////////////////////
  // Assemble initial quaternion through GTSAM constructor
  // ::quaternion(w,x,y,z);
  Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
                                         initial_state(4), initial_state(5));
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.tail<3>());

  imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias


  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);
  // Assemble prior noise model and add it the graph.`


  // Add all prior factors (pose, velocity, bias) to the graph.
  
  graph->addPrior(X(correction_count), prior_pose, pose_noise_model);
  graph->addPrior(V(correction_count), prior_velocity, velocity_noise_model);
  graph->addPrior(B(correction_count), prior_imu_bias, bias_noise_model);
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  auto p = imuParams();

  preintegrated =
      std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);

  assert(preintegrated);

  // Store previous state for imu integration and latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;
  prevStateOdom=prev_state;
  prevBiasOdom=prior_imu_bias;
  // Keep track of total error over the entire run as simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  double output_time = 0.0;
  double dt = 0.001;  // The real system has noise, but here, results are nearly
                      // exactly the same, so keeping this for simplicity.




/*
  //////////////////////////////////Loop Startuing/////////////////////////////////////////////
  // All priors have been set up, now iterate through the data file.
  while (file.good()) {
    // Parse out first value
    getline(file, value, ',');
    int type = stoi(value.c_str());

    if (type == 0) {  // IMU measurement
      Vector6 imu;
      for (int i = 0; i < 5; ++i) {
        getline(file, value, ',');
        imu(i) = stof(value.c_str());
      }
      getline(file, value, '\n');
      imu(5) = stof(value.c_str());

      // Adding the IMU preintegration.
      preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

    } else if (type == 1) {  // GPS measurement
      Vector7 gps;
      for (int i = 0; i < 6; ++i) {
        getline(file, value, ',');
        gps(i) = stof(value.c_str());
      }
      getline(file, value, '\n');
      gps(6) = stof(value.c_str());

      correction_count++;

      // Adding IMU factor and GPS factor and optimizing.
      auto preint_imu =
          dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
      ImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1),
                           X(correction_count), V(correction_count),
                           B(correction_count - 1), preint_imu);
      graph->add(imu_factor);
      imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
      graph->add(BetweenFactor<imuBias::ConstantBias>(
          B(correction_count - 1), B(correction_count), zero_bias,
          bias_noise_model));

      auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);
      GPSFactor gps_factor(X(correction_count),
                           Point3(gps(0),   // N,
                                  gps(1),   // E,
                                  gps(2)),  // D,
                           correction_noise);
      graph->add(gps_factor);

      // Now optimize and compare results.
      prop_state = preintegrated->predict(prev_state, prev_bias);
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prev_bias);

      Values result;

      if (use_isam) {
        isam2->update(*graph, initial_values);
        isam2->update();
        result = isam2->calculateEstimate();

        // reset the graph
        graph->resize(0);
        initial_values.clear();

      } else {
        LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
        result = optimizer.optimize();
      }

      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      // Reset the preintegration object.
      preintegrated->resetIntegrationAndSetBias(prev_bias);

      // Print out the position and orientation error for comparison.
      Vector3 gtsam_position = prev_state.pose().translation();
      Vector3 position_error = gtsam_position - gps.head<3>();
      current_position_error = position_error.norm();

      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
      Quaternion quat_error = gtsam_quat * gps_quat.inverse();
      quat_error.normalize();
      Vector3 euler_angle_error(quat_error.x() * 2, quat_error.y() * 2,
                                quat_error.z() * 2);
      current_orientation_error = euler_angle_error.norm();

      // display statistics
      cout << "Position error:" << current_position_error << "\t "
           << "Angular error:" << current_orientation_error << "\n";

      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              output_time, gtsam_position(0), gtsam_position(1),
              gtsam_position(2), gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(),
              gtsam_quat.w(), gps(0), gps(1), gps(2), gps_quat.x(),
              gps_quat.y(), gps_quat.z(), gps_quat.w());

      output_time += 1.0;

    } else {
      cerr << "ERROR parsing file\n";
      return 1;
    }
  }
  fclose(fp_out);
  cout << "Complete, results written to " << output_filename << "\n\n";
*/
  ros::spin();
  return 0;
}
