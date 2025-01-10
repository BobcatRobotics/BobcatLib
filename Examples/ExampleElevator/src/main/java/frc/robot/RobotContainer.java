// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Controllers.OI;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final OI s_Controls;
  public final ElevatorSubsystem elevator;
  public RobotContainer() {
    s_Controls = new OI();
    elevator = new ElevatorSubsystem();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Command raiseElevator = Commands.run(()-> elevator.moveElevator(new Rotation2d(50*360)),elevator);
    Command lowerElevator = Commands.run(()-> elevator.moveElevator(new Rotation2d(0)),elevator);
    Command nextSetPoint = Commands.run(()-> elevator.moveElevatorToNext(),elevator);
    Command holdPosition = Commands.run(()-> elevator.holdPosition());
    Command stopCommand = Commands.run(()-> elevator.holdPosition(),elevator);
    s_Controls.robotCentric.whileTrue(nextSetPoint).onFalse(stopCommand);
    s_Controls.dpadForwardBtn.whileTrue(raiseElevator).onFalse(holdPosition);
    s_Controls.dpadBackBtn.whileTrue(lowerElevator).onFalse(holdPosition);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
