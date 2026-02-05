// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.SwerveDriveTeleop;
import frc.robot.io.IO;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import frc.robot.commands.flywheel.RunFlywheel;

public class RobotContainer {

    // The robot's subsystems

  public final DriveSubsystem drivetrain = new DriveSubsystem();
  public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  public final LimelightSubsystem limelight = new LimelightSubsystem();
  // public final IntakeSubsystem intake = new IntakeSubsystem();

//  private final SendableChooser<Command> autoChooser;
  protected SendableChooser<Alliance> allianceColor = new SendableChooser<>();

  private final Field2d field;

  public RobotContainer() {
    configureBindings();

 //   autoChooser = AutoBuilder.buildAutoChooser("None");

        // Configure default commands
    this.drivetrain.setDefaultCommand(
        // IMPLEMENT DEFAULT COMMAND
        new SwerveDriveTeleop(this.drivetrain));
    RunFlywheel flywheelCommand = new RunFlywheel(flywheel, 30);
    this.flywheel.setDefaultCommand(flywheelCommand);
    //this.flywheel.setDefaultCommand(
      //new RunFlywheel(this.flywheel, 10));

    

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    DataLogManager.start();
    DataLogManager.logNetworkTables(true);

  }

  private void configureBindings() {
  IO io = new IO();

    io.init(this);


  }

  public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");

//    return autoChooser.getSelected();
  }
}
