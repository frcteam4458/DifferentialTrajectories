#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/simulation/PWMSim.h>

#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>

#include <frc/AnalogGyro.h>
#include <frc/simulation/AnalogGyroSim.h>

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include <units/velocity.h>
#include <units/angle.h>

#include <frc/smartdashboard/Field2d.h>

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  void DriveVolts(units::volt_t left, units::volt_t right);
  void Drive(units::meters_per_second_t fwd, units::meters_per_second_t omega);

  units::degree_t GetAngle();

  frc::Pose2d GetPose();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

 private:
  frc::PWMSparkMax fl;
  frc::PWMSparkMax fr;
  frc::PWMSparkMax bl;
  frc::PWMSparkMax br;

  frc::sim::PWMSim s_fl;
  frc::sim::PWMSim s_fr;
  frc::sim::PWMSim s_bl;
  frc::sim::PWMSim s_br;

  frc::Encoder flEncoder;
  frc::Encoder frEncoder;
  frc::Encoder blEncoder;
  frc::Encoder brEncoder;

  frc::sim::EncoderSim s_flEncoder;
  frc::sim::EncoderSim s_frEncoder;
  frc::sim::EncoderSim s_blEncoder;
  frc::sim::EncoderSim s_brEncoder;

  frc::AnalogGyro gyro;
  frc::sim::AnalogGyroSim s_gyro;

  frc::DifferentialDriveOdometry odometry;

  frc::sim::DifferentialDrivetrainSim drivetrain;

  frc::Field2d field;
};
