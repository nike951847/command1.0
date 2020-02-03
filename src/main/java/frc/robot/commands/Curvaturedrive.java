/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Curvaturedrive extends CommandBase {
  
  private final Drivetrain m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;
  private BooleanSupplier m_qickturn;
  /**
   * 
   * Creates a new Curvaturedrive.
   */
  public Curvaturedrive(Drivetrain subsystem, DoubleSupplier forward, DoubleSupplier rotation,BooleanSupplier Qturn) {
    m_drive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    m_qickturn = Qturn;
    addRequirements(m_drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.curvatureDrive(m_forward.getAsDouble(),m_rotation.getAsDouble(),  m_qickturn.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
