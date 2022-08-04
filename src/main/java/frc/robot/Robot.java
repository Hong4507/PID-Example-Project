// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  final double kEncoderCPR = 2048;
  final double kWheelDiameterMeters = 0.1524;
  final double kWheelCircumference = kWheelDiameterMeters*Math.PI;
  final double kGearRatio = 10.7142857;
  final double distancePerPulse = kWheelCircumference/kEncoderCPR/kGearRatio/ Math.sqrt(2)*10;

  final double kWheelBase = 0.4904;
  final double kTrackWidth = 0.5682;

  Joystick controller = new Joystick(0);

  final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(new Translation2d(kWheelBase/2, kTrackWidth/2), new Translation2d(kWheelBase/2, -kTrackWidth/2), new Translation2d(-kWheelBase/2, kTrackWidth/2), new Translation2d(-kWheelBase,-kTrackWidth));

  WPI_TalonFX motorFL = new WPI_TalonFX(1);
  WPI_TalonFX motorRL = new WPI_TalonFX(2);
  WPI_TalonFX motorFR = new WPI_TalonFX(3);
  WPI_TalonFX motorRR = new WPI_TalonFX(4);

  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  MecanumDrive mDrive = new MecanumDrive(motorFL,motorRL,motorFR,motorRR);

  MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, m_gyro.getRotation2d());

  double targetPosition = 3;
  double errorSum;
  double lastTimeStamp;
  double lastPosition;
  
  double KP = 0.249;
  double KI = 0.01;
  double KD = 0.0862;
  double iLimit = 0.225;
  
  @Override
  public void robotInit() {
    motorRR.setInverted(true);
    motorFR.setInverted(true);
    motorFL.setInverted(false);
    motorRL.setInverted(false);
  }

  @Override
  public void robotPeriodic() {
    odometry.update(m_gyro.getRotation2d(), new MecanumDriveWheelSpeeds(
      motorFL.getSelectedSensorVelocity()*distancePerPulse,
      motorFR.getSelectedSensorVelocity()*distancePerPulse,
      motorRL.getSelectedSensorVelocity()*distancePerPulse,
      motorRR.getSelectedSensorVelocity()*distancePerPulse
    ));
  }

  @Override
  public void autonomousInit() {
    resetOdometry();
  }

  @Override
  public void autonomousPeriodic() {
    double dT = Timer.getFPGATimestamp() - lastTimeStamp;
    double currentPosition = -odometry.getPoseMeters().getX();
    double error = targetPosition-currentPosition;
    double rate = (currentPosition-lastPosition)/dT;

    if(Math.abs(error)<iLimit){
      errorSum += error*dT;
    }
    double output = KP*error + KI*errorSum + KD*rate;

    DriverStation.reportWarning(error+"", false);
    
    mDrive.driveCartesian(output, 0, 0);

    lastTimeStamp = Timer.getFPGATimestamp();
    lastPosition = currentPosition;

    SmartDashboard.putNumber("position", currentPosition);
  }

  @Override
  public void teleopInit() {
    resetOdometry();
  }

  @Override
  public void teleopPeriodic() {
    mDrive.driveCartesian(-0.4*controller.getRawAxis(1), 0.4*controller.getRawAxis(0), 0.3*controller.getRawAxis(4));
    SmartDashboard.putNumber("position", odometry.getPoseMeters().getX());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    resetOdometry();
  }

  @Override
  public void testPeriodic() {
    mDrive.driveCartesian(0.1, 0, 0);
    DriverStation.reportWarning(odometry.getPoseMeters().getX()+"", false);
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void resetEncoders(){
    motorFL.setSelectedSensorPosition(0);
    motorRL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorRR.setSelectedSensorPosition(0);
  }
  public void resetOdometry(){
    resetEncoders();
    odometry.resetPosition(new Pose2d(new Translation2d(0,0),new Rotation2d(0)), new Rotation2d(0));
  }
}