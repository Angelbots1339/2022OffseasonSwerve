// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.VisonConstants.*;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.Swerve;


public class PidToTarget extends CommandBase {
  Swerve swerve;
  /** Creates a new PidToTarget. */
  public PidToTarget(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  //TODO test output of camera and make sure units are aproprite
  @Override
  public void execute() {
    if(CAMERA.getLatestResult().hasTargets()){
      PhotonTrackedTarget target = CAMERA.getLatestResult().getBestTarget();
      Translation2d translation = new Translation2d(target.getCameraToTarget().getX(), target.getCameraToTarget().getY());
    
      swerve.angularDrive(translation, Rotation2d.fromDegrees(target.getYaw()), false, true);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
