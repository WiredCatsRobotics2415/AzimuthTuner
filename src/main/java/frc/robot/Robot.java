// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.subsystems.Module;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Robot extends TimedRobot {

  private final int TUNE_MOTOR = 0; //0 - front left, 1 - front right, 2 - back left, 3 - back right

  SendableChooser<Command> routineChooser;
  frc.subsystems.Module module;

  @Override
  public void robotInit() {
    module = new Module(RobotMap.AZIMUTH_PORTS[TUNE_MOTOR], RobotMap.ABSOLUTE_ENCODER_PORTS[TUNE_MOTOR]);
    routineChooser = new SendableChooser<Command>();
    routineChooser.setDefaultOption("Slow (Quasistatic) forward", module.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    routineChooser.addOption("Slow (Quasistatic) backward", module.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    routineChooser.addOption("Fast (Dynamic) forward", module.sysIdDynamic(SysIdRoutine.Direction.kForward));
    routineChooser.addOption("Slow (Dynamic) backward", module.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putData("Choose Test", routineChooser);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
      routineChooser.getSelected().schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
}
