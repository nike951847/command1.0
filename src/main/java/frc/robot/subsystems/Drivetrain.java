/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;


public class Drivetrain extends SubsystemBase {
  private Encoder leftencoder =
  new Encoder(DriverConstants.kLeftEncoderPorts[1],DriverConstants.kLeftEncoderPorts[2],DriverConstants.kLeftEncoderReversed);
  private Encoder rightencoder =
  new Encoder(DriverConstants.kRightEncoderPorts[1],DriverConstants.kRightEncoderPorts[2]);

  WPI_VictorSPX left1motor  = new WPI_VictorSPX(DriverConstants.leftmotor1ID);
  WPI_VictorSPX left2motor  = new WPI_VictorSPX(DriverConstants.leftmotor2ID);
  WPI_VictorSPX right1motor = new WPI_VictorSPX(DriverConstants.rightmotor1ID);
  WPI_VictorSPX right2motor = new WPI_VictorSPX(DriverConstants.rightmotor2ID);
  SpeedControllerGroup m_left = new SpeedControllerGroup(left1motor, left2motor);
  SpeedControllerGroup m_right= new SpeedControllerGroup(right1motor, right2motor);
  DifferentialDrive drive = new DifferentialDrive(m_left,m_right);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  
  private final AHRS ahrs = new AHRS();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public Drivetrain(){
    leftencoder.setDistancePerPulse(DriverConstants.kEncoderDistancePerPulse);
    rightencoder.setDistancePerPulse(DriverConstants.kEncoderDistancePerPulse);
    //
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }
  public void curvatureDrive(double forward, double rotation,boolean qickturn) {
   drive.curvatureDrive(forward, rotation, qickturn);
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftencoder.getDistance(),
                      rightencoder.getDistance());
                      var translation = m_odometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("X",translation.getX());
    SmartDashboard.putNumber("Y",translation.getY());
    // This method will be called once per scheduler run
  }
  /**
   * Creates a new DriveSubsystem.
   */

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftencoder.getRate(), rightencoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftencoder.reset();
    rightencoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftencoder.getDistance() + rightencoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftencoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightencoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (DriverConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate() * (DriverConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
  
