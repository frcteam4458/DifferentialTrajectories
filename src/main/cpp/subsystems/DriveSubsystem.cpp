#include "subsystems/DriveSubsystem.h"

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <wpi/math>

DriveSubsystem::DriveSubsystem() :
  fl{FRONT_LEFT},
  fr{FRONT_RIGHT},
  bl{BACK_LEFT},
  br{BACK_RIGHT},

  s_fl{FRONT_LEFT},
  s_fr{FRONT_RIGHT},
  s_bl{BACK_LEFT},
  s_br{BACK_RIGHT},

  flEncoder{FRONT_LEFT_ENCODER[0], FRONT_LEFT_ENCODER[1]},
  frEncoder{FRONT_RIGHT_ENCODER[0], FRONT_RIGHT_ENCODER[1]},
  blEncoder{BACK_LEFT_ENCODER[0], BACK_LEFT_ENCODER[1]},
  brEncoder{BACK_RIGHT_ENCODER[0], BACK_RIGHT_ENCODER[1]},

  s_flEncoder{flEncoder},
  s_frEncoder{frEncoder},
  s_blEncoder{blEncoder},
  s_brEncoder{frEncoder},

  gyro{0},

  s_gyro{gyro},

  odometry{frc::Rotation2d{0_deg}, frc::Pose2d{frc::Translation2d{0_m, 0_m}, frc::Rotation2d{0_deg}}},

  drivetrain{ // again, stole the construction off of wpilib docs
    frc::DCMotor::CIM(2),
    7.29,
    7.5_kg_sq_m,
    60_kg,
    3_in,
    0.7112_m,
    {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}
  }
{
  fl.SetInverted(true);
  bl.SetInverted(true);

  flEncoder.SetReverseDirection(true);
  blEncoder.SetReverseDirection(true);

  s_flEncoder.SetCount(1000);
  s_frEncoder.SetCount(1000);
  s_blEncoder.SetCount(1000);
  s_brEncoder.SetCount(1000);

  // 0.0762 is the wheel radius in meters
  flEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_flEncoder.GetCount());
  frEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_frEncoder.GetCount());
  blEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_blEncoder.GetCount());
  brEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_brEncoder.GetCount());
}

void DriveSubsystem::Periodic() {
  frc::DifferentialDriveWheelSpeeds wheelSpeeds{units::meters_per_second_t{flEncoder.GetRate()},
    units::meters_per_second_t{frEncoder.GetRate()}};
  odometry.Update(frc::Rotation2d{GetAngle()}, units::meter_t{flEncoder.GetDistance()}, units::meter_t{frEncoder.GetDistance()});  
  field.SetRobotPose(odometry.GetPose());
  frc::SmartDashboard::PutData("Field", &field);
}

void DriveSubsystem::SimulationPeriodic() {
  drivetrain.SetInputs(fl.Get() * 12.0_V, fr.Get() * 12.0_V);

  drivetrain.Update(20_ms);

  s_flEncoder.SetDistance(drivetrain.GetLeftPosition().value());
  s_flEncoder.SetRate(drivetrain.GetLeftVelocity().value());
  s_frEncoder.SetDistance(drivetrain.GetRightPosition().value());
  s_frEncoder.SetRate(drivetrain.GetRightVelocity().value());
  s_blEncoder.SetDistance(drivetrain.GetLeftPosition().value());
  s_blEncoder.SetRate(drivetrain.GetLeftVelocity().value());
  s_brEncoder.SetDistance(drivetrain.GetRightPosition().value());
  s_brEncoder.SetRate(drivetrain.GetRightVelocity().value());

  s_gyro.SetAngle(-drivetrain.GetHeading().Degrees().value());
}

void DriveSubsystem::DriveVolts(units::volt_t left, units::volt_t right) {
  fl.SetVoltage(left);
  fr.SetVoltage(right);
  bl.SetVoltage(left);
  br.SetVoltage(right);
}

void DriveSubsystem::Drive(units::meters_per_second_t fwd, units::meters_per_second_t omega) {

}

units::degree_t DriveSubsystem::GetAngle() {
  return units::degree_t{gyro.GetAngle()};
}

frc::Pose2d DriveSubsystem::GetPose() {
  return odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t{flEncoder.GetRate()}, units::meters_per_second_t{frEncoder.GetRate()}};
}