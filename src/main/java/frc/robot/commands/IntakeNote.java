// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANLauncher;
import static frc.robot.Constants.LauncherConstants.*;

public class IntakeNote extends Command {
  CANLauncher launcher;
  /** Creates a new IntakeNote. */
  public IntakeNote(CANLauncher launcher) {
    this.launcher = launcher;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.launcher.setLaunchWheel(kIntakeLauncherSpeed);
    this.launcher.setFeedWheel(kIntakeFeederSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.launcher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
