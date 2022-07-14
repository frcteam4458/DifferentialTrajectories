#include "RobotContainer.h"

#include "Constants.h"

RobotContainer::RobotContainer() :
  driveSubsystem{},

  trajectoryConfig{MAX_SPEED, MAX_ACCEL},

  autoCommand{
    autoTrajectory,
    [this]() { return driveSubsystem.GetPose(); },
    frc::RamseteController{ramseteB, ramseteZeta},
    frc::SimpleMotorFeedforward<units::meters>{kS, kV},  
    frc::DifferentialDriveKinematics{units::meter_t{WIDTH}},
    [this] { return driveSubsystem.GetWheelSpeeds(); },
    frc::PIDController{0, 0, 0},
    frc::PIDController{0, 0, 0},
    [this](auto l, auto r) {   
      driveSubsystem.DriveVolts(units::volt_t{l.value()}, units::volt_t{r.value()});
    },
    {&driveSubsystem}    
  }

 {
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return &autoCommand;
}
