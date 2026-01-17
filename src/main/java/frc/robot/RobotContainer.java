// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drivetrain.SwerveDriveTeleop;
import frc.robot.io.IO;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import frc.robot.subsystems.QuestNav.QuestNavSubsystem;

public class RobotContainer {

    // The robot's subsystems

  public final DriveSubsystem drivetrain = new DriveSubsystem();
  public final LimelightSubsystem limelight = new LimelightSubsystem();
  public final QuestNavSubsystem questnav = new QuestNavSubsystem(drivetrain);

  private final SendableChooser<Command> autoChooser;
  protected SendableChooser<Alliance> allianceColor = new SendableChooser<>();

  private final Field2d field;

  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("None");

        // Configure default commands
    this.drivetrain.setDefaultCommand(
        // IMPLEMENT DEFAULT COMMAND
        new SwerveDriveTeleop(this.drivetrain));

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
    return autoChooser.getSelected();
  }
}
