#include <atomic>
#include <cmath>
#include <csignal>
#include <memory>
#include <string>

#include <elmo_ethercat/ElmoEthercat.hpp>
#include <elmo_ethercat/ElmoEthercatMaster.hpp>

#include <any_worker/Worker.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

using namespace elmo;
using namespace ethercat;

#define _USE_MATH_DEFINES
#define TIME_STEP 0.0025

std::atomic<bool> sigint{false};
std::string setupFile;
const double timeStep = 0.0025;
std::vector<ElmoEthercatPtr> elmodrives;
std::shared_ptr<ElmoEthercatMaster> master;
Command command;
unsigned int counter = 0;

std::chrono::duration<double, std::milli> elapsed;
double velocity_ = 0;
double torque_ = 0;
double position_ = 0;

bool updateController();
void veGet(const std_msgs::Float32 &msg){
    velocity_ = msg.data;
}
void torGet(const std_msgs::Float32 &msg){
    torque_ = msg.data;
}
void posGet(const std_msgs::Float32 &msg){
    position_ = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elmo_control");
  ros::NodeHandle nh;

  ros::Subscriber vel_sub = nh.subscribe("/elmo_velocity",10,veGet);
  ros::Subscriber torque_sub = nh.subscribe("/elmo_torque",10,torGet);
  ros::Subscriber position_sub = nh.subscribe("/elmo_position",10,posGet);

  std::cout << "Starting in 3 seconds.\nGrab your emergency stop button!" << std::endl;
  for (int i = 3; i > 0; i--) {
    std::cout << "Starting in: " << i << "s" << std::endl;
    usleep(1000000);
  }

  if (argc > 1) {
    setupFile = argv[1];
  } else {
    std::cout << "Pass path to config file as command line argument:\n" << argv[0] << " path/to/configfile.yaml" << std::endl;
    return 1;
  }

  const bool standalone = false;
  const bool installSignalHandler = false;  // Signals are handled manually
  master = std::make_shared<ElmoEthercatMaster>(standalone, installSignalHandler, timeStep);
  if (!master->loadSetup(setupFile)) {
    MELO_ERROR_STREAM("ElmoEthercat Master could not load setup parameters.");
  }
  elmodrives = master->getElmodrives();

  if(master->startup()){
    MELO_INFO_STREAM("Start the master successfully----startup");
  }
  else{
    MELO_ERROR_STREAM("Mater can not be started smoothly, please check the connection of elmos.");
  }


  ros::Rate loop_rate(1./TIME_STEP);
  while (ros::ok())
  {
    //Count the processing time.
    auto start = std::chrono::high_resolution_clock::now();

    //main function
    updateController();

    ros::spinOnce();
    loop_rate.sleep();

    //Finish Code to be timed.
    auto end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
  }
}

bool updateController() {
  command.setModeOfOperation(master->modeofOperation_); 
  //CyclicSynchronousVelocityMode CyclicSynchronousTorqueMode CyclicSynchronousPositionMode
  static bool operationInitiallyEnabled = false;
  if (!operationInitiallyEnabled) {
    master->setDriveStateForAllSlaves(DriveState::OperationEnabled, false);
    operationInitiallyEnabled = true;
  }

  if (master->allDrivesChangedStateSuccessfully()) {
    if (!master->allDrivesAreInTheState(DriveState::OperationEnabled)) {
      MELO_ERROR_STREAM(
          "CST example: Fault occurred, not al slaves are in "
          "operation enabled mode");
    } else {

    //show the current status of motor, including position, velocity and torque.
      static double position = 0;
      if (counter % 10 == 0) {
        for (const auto& elmodrive : elmodrives) {
          const double current = elmodrive->getReading().getActualCurrent();
          const double torque = elmodrive->getReading().getActualTorque();
          const double velocity = elmodrive->getReading().getActualVelocity();
          position = elmodrive->getReading().getActualPosition();

          MELO_INFO_STREAM("ELMO drive reading of \"" << elmodrive->getNameOfSlave() << "\":");
          MELO_INFO_STREAM("  Current:     " << current << " A");
          MELO_INFO_STREAM("  Torque:      " << torque << " Nm");
          MELO_INFO_STREAM("  Velocity:    " << velocity << " rad/s");
          MELO_INFO_STREAM("  Position:    " << position << " rad");
          MELO_INFO_STREAM("  Elapsed time: " << elapsed.count() << "ms");
        }
      }

      command.setTargetTorque(torque_);
      command.setTargetVelocity(velocity_);
      command.setTargetPosition(position_);
      for (const auto& elmodrive : elmodrives) {
        elmodrive->stageCommand(command);
      }
    }
  }
  else{
    MELO_ERROR_STREAM("Not all of the drivers have been setup well.");
  }
  master->update();
  counter++;
  return true;
  }  