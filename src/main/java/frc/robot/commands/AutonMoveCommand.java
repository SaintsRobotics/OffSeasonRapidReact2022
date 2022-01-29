// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveDrivetrainHardware;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutonMoveCommand extends CommandBase {

  SwerveDriveSubsystem m_swerveSubsystem;
  double xSpeed;
  double ySpeed;
  double rotSpeed;

  PIDController xPID = new PIDController(0.3, 0, 0);
  PIDController yPID = new PIDController(0.3, 0, 0);
  PIDController rotPID = new PIDController(0.3, 0, 0);

  Pose2d currentPose = new Pose2d();

  /** Creates a new AutonCommand. */
  public AutonMoveCommand(SwerveDriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = subsystem;
    addRequirements(subsystem);
    xPID.setTolerance(0.1);
    yPID.setTolerance(0.1);
    rotPID.setTolerance(Math.PI/18);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = m_swerveSubsystem.getPose();
    xPID.setSetpoint(currentPose.getX());
    yPID.setSetpoint(currentPose.getY());
    rotPID.setSetpoint(currentPose.getRotation().getRadians());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = m_swerveSubsystem.getPose();
    xSpeed = xPID.calculate(currentPose.getX());
    ySpeed = yPID.calculate(currentPose.getY());
    rotSpeed = rotPID.calculate(currentPose.getRotation().getRadians());

    m_swerveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint());
  }


  public AutonMoveCommand moveForwardBy(double x){ // change robot relative X
    xPID.setSetpoint(Math.cos(currentPose.getRotation().getRadians()) * x);
    yPID.setSetpoint(Math.sin(currentPose.getRotation().getRadians()) * x);
    return this; 
  }
  public AutonMoveCommand moveSidewaysBy(double y){ // change robot relative Y (left is positive, right is negative)
    xPID.setSetpoint(Math.sin(currentPose.getRotation().getRadians()) * y);
    yPID.setSetpoint(Math.cos(currentPose.getRotation().getRadians()) * y);
    return this; 
  }
  public AutonMoveCommand changeXPosBy(double x){ // change field relative X
    xPID.setSetpoint(x + m_swerveSubsystem.getPose().getX());
    return this; 
  }
  public AutonMoveCommand changeYPosBy(double y){ // change field relative Y
    yPID.setSetpoint(y + m_swerveSubsystem.getPose().getY());
    return this; 
  }
  public AutonMoveCommand changeHeadingBy(double rot){ // change robot heading
    rotPID.setSetpoint(Math.toRadians(rot) + m_swerveSubsystem.getPose().getRotation().getRadians());
    return this; 
  }
  public AutonMoveCommand setXTo (double x) { // set specific X value
    xPID.setSetpoint(x);
    return this;
  }
  public AutonMoveCommand setYTo (double y) { // set specific Y value
    yPID.setSetpoint(y);
    return this;
  }
  public AutonMoveCommand setHeadingTo (double rot) { // set specific heading 
    rotPID.setSetpoint(Math.toRadians(rot));
    return this;
  }
}
