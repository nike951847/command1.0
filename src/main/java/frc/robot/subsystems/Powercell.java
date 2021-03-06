/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Powercell extends SubsystemBase {
  /**
   * Creates a new Powercell.
   */
  private WPI_VictorSPX intake = new WPI_VictorSPX(5);
  private WPI_VictorSPX flywheel = new WPI_VictorSPX(6);
  private WPI_VictorSPX feed = new WPI_VictorSPX(7);

  /**
   * Creates a new ball.
   */

  public Powercell() {
    flywheel.setNeutralMode(NeutralMode.Coast);

  }
  public void intake(){
    intake.set(ControlMode.PercentOutput,0.5);


  }
  public void intakestop(){
    intake.set(ControlMode.PercentOutput,0);
  }
  public void flywheelspinup(){
    flywheel.set(ControlMode.PercentOutput,1);
  }
  public void flywheelslowdown(){
    flywheel.set(ControlMode.PercentOutput,0);
  }
  public void feed(){

    /**將球傳往射球 */
    feed.set(ControlMode.PercentOutput, 0.5);


  }
  public void feedstop(){
    feed.set(ControlMode.PercentOutput, 0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler
}
}