#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

/*
 all these functions are pretty self explanatory
 init calls when someyhing starts, periodic is every 20ms (optimally, if it takes longer thats really bad and means
 that your code sucks)
*/

/*
  this code is command based, so basically you have subsystems that interface with the robot itself, commands doing basic
  actions using the methods those subsystems lay out, then you can have command groups to have advanced functions
  
  this file is supposed to be incredibly basic, all commands/subsystems are to be made and initialized in the
  RobotContainer files

  declarative progtsmming
*/
void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
