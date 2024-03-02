// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chuck;

public class chuckFireCmd extends CommandBase {
  private Chuck m_chuck;
  private final double m_timeout;
  private final Timer m_timer = new Timer();

  public chuckFireCmd(Chuck chuck, double timeout) {
    m_chuck = chuck;
    m_timeout = timeout;
    addRequirements(chuck);
  }

  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  public void execute() {
    m_chuck.SpeakerShoot2();
  }

  public void end(boolean interrupted) {
    m_chuck.stopRunLower();
  }

  public boolean isFinished() {
    return m_timer.get() >= m_timeout;
  }
}
