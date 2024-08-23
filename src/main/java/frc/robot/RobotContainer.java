// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.OperatorConstants.*;


@SuppressWarnings("unused")

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem(32);
  private final ShooterSubsystem testMotorSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    Trigger leftTrigger = driverController.leftTrigger();
    Trigger rightTrigger = driverController.rightTrigger();
    Trigger bButton = driverController.b();
    Trigger xButton = driverController.x();
    leftTrigger.whileTrue(Commands.runEnd(
      () -> testMotorSubsystem.setIndexerMotorDutyCycle(driverController.getLeftTriggerAxis() * -0.5), 
      () -> testMotorSubsystem.setIndexerMotorDutyCycle(0)
    ));
    rightTrigger.whileTrue(Commands.runEnd(
      () -> testMotorSubsystem.setIndexerMotorDutyCycle(driverController.getRightTriggerAxis() * 0.5), 
      () -> testMotorSubsystem.setIndexerMotorDutyCycle(0)
    ));
    bButton.onTrue(Commands.runOnce(() -> testMotorSubsystem.setShooterMotorDutyCycle(1)));
    bButton.onFalse(Commands.runOnce(() -> testMotorSubsystem.setShooterMotorDutyCycle(0)));
    
    /*
    //Right Joystick to control speed
    Trigger rightJoystickY = m_driverController
            .axisGreaterThan(XboxController.Axis.kRightY.value, XBOX_JOYSTICK_THRESHOLD)
            .or(m_driverController.axisLessThan(XboxController.Axis.kRightY.value, -XBOX_JOYSTICK_THRESHOLD));
    rightJoystickY.whileTrue(Commands.runEnd(() -> {
      m_exampleSubsystem.setMotor(-m_driverController.getRightY());
    }, () -> {
      m_exampleSubsystem.setMotor(0);
    }, m_exampleSubsystem));
    */
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
