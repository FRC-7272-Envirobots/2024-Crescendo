// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANLauncher extends SubsystemBase {
  CANSparkMax m_launchWheel;
  CANSparkMax m_feedWheel;

  /** Creates a new Launcher. */
  public CANLauncher() {
    m_launchWheel = new CANSparkMax(kLauncherID, MotorType.kBrushed);
    m_feedWheel = new CANSparkMax(kFeederID, MotorType.kBrushed);

    // m_launchWheel.setSmartCurrentLimit(kLauncherCurrentLimit);
    // m_feedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel.set(0);
    m_feedWheel.set(0);
  }
}
