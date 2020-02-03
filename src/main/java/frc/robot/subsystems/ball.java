/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;


public class ball extends SubsystemBase {
  private WPI_VictorSPX intake   = new WPI_VictorSPX(5); 
  private WPI_VictorSPX flywheel = new WPI_VictorSPX(6);

  /**
   * Creates a new ball.
   */

  public ball() {

  }
  public void intake(){
    intake.set(ControlMode.PercentOutput,0.5);

  }
  public void flywheelspeedup(){

    


  }
  public void flywheelslowdown(){

  }
  public void upward(){
    /**將球傳往射球 */

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
