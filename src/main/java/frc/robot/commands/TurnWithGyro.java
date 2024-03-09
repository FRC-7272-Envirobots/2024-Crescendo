// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.units.*;
// import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDrivetrain;

public class TurnWithGyro extends Command {
  private final CANDrivetrain m_drive;
  private final double m_speed;
  double m_targetAngle = 0;
  /** Creates a new ForwardTurnForward. */
  public TurnWithGyro(CANDrivetrain drivetrain, double speed, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drivetrain;
    m_speed = speed;
    m_targetAngle = targetAngle;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_speed);
    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(m_drive.getGyroAngle())>= m_targetAngle;
    }

}
