#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/Subsystem.h>

#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/RamseteCommand.h>

#include "subsystems/DriveSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  DriveSubsystem driveSubsystem;

  frc::TrajectoryConfig trajectoryConfig;
  frc::Trajectory autoTrajectory;
  frc2::RamseteCommand autoCommand;

  void ConfigureButtonBindings();
};
