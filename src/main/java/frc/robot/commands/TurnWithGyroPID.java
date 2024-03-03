// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.units.*;
// import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDrivetrain;

public class TurnWithGyroPID extends Command {
  private final CANDrivetrain m_drive;
  private final double m_speed;
  double m_targetAngle = 0;
  PIDController pid;
  /** Creates a new ForwardTurnForward. */
  public TurnWithGyroPID(CANDrivetrain drivetrain, double speed, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drivetrain;
    m_speed = speed;
    m_targetAngle = targetAngle;
    pid = new PIDController(.05, .03, .005);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetGyro();
    pid.reset();
    pid.setIntegratorRange(-0.05, 0.05);
    pid.setSetpoint(m_targetAngle);
    pid.setTolerance(-1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, pid.calculate(-m_drive.getGyroAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_drive.getGyroAngle());
    System.out.println(pid.getSetpoint());
    return pid.atSetpoint();
    //return false;
    }

}
