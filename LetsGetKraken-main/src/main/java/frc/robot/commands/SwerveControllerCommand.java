// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class SwerveControllerCommand extends Command {
 
private final SwerveSubsystem swerveSubsystem;
private final Supplier<Double> xspeed, yspeed, rotspeed;
private final Supplier<Boolean> fieldOrientedFunction;
private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;


  public SwerveControllerCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xspeed, Supplier<Double> yspeed, Supplier<Double> rotspeed, Supplier<Boolean> fieldOrientedFunction){

      this.swerveSubsystem = swerveSubsystem;
      this.xspeed = xspeed;
      this.yspeed = yspeed;
      this.rotspeed = rotspeed;
      this.fieldOrientedFunction = fieldOrientedFunction;
      this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kStickSlewLimitAccelerator);
      this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kStickSlewLimitAccelerator);
      this.rotLimiter = new SlewRateLimiter(Constants.DriveConstants.kStickSlewLimitAccelerator);
      addRequirements(swerveSubsystem);
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   
  public SwerveControllerCommand(SwerveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = xspeed.get(); //Update speeds with supplier that gets joystick from outside
    double ySpeed = yspeed.get();
    double rotSpeed = rotspeed.get();

    //DEADBAND CODE
    xSpeed = Math.abs(xSpeed) > Constants.DriveConstants.kStickDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > Constants.DriveConstants.kStickDeadband ? ySpeed : 0;
    rotSpeed = Math.abs(rotSpeed) > Constants.DriveConstants.kStickDeadband ? rotSpeed : 0;

    //MAKE DRIVE SMOOTH

    xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kMaxMetersPerSecondTeleop;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kMaxMetersPerSecondTeleop;
    rotSpeed = rotLimiter.calculate(rotSpeed) * Constants.DriveConstants.kMaxRadiansPerSecondTeleop;

    //CONSTRUCT DESIRED CHASIS SPEEDS
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {//WHY DOES ChassisSpeeds below have to be capitolized rather than chasisSpeeds.fromFieldRelative...
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getRotation2d()); //CAN SWITCH THIS TO NOT USE UNITS AND JUST HAVE 0-1 I THINK

    } else {
      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getRotation2d());

    }

    //CONVERTING CHASSIS SPEEDS TO EACH MOD
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //OUTPUT TO WHEELS
    swerveSubsystem.setModuleStates(moduleStates);
  }

  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
