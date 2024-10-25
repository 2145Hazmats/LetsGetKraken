package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;

  //private final CANcoder driveEncoder;
  //private final CANcoder turningEncoder;

  private final PIDController turningPIDController; //or you could use onboard! Pretty sure we can run 2 channels of PID on Krakens

  private final CANcoder absoluteEncoder;  //plugged into robo rio
  private final boolean isAbsoluteEncoderReversed;
  private final double absoluteEncoderOffsetRAD;

  public SwerveModule(int driveMotorID, int turningMotorID, boolean isDriveMotorReversed, boolean isTurningMotorReversed,
          int absoluteEncoderID, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed) {
        
        absoluteEncoderOffsetRAD = absoluteEncoderOffset; 
          //why does tutorial do this.absoluteEncoderOffsetRAD = absoluteEncoderOffset'?
          // why does tutorial say  this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderID);
        
        driveMotor = new TalonFX(driveMotorID);
        turningMotor = new TalonFX(turningMotorID);

        driveMotor.setInverted(isDriveMotorReversed);
        turningMotor.setInverted(isTurningMotorReversed);

        turningPIDController = new PIDController(Constants.SwerveModConstants.kturingMotorP,Constants.SwerveModConstants.kturingMotorI,Constants.SwerveModConstants.kturingMotorD);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);  //Im assuming that -180 degrees is 360 degrees away from 180. Why is not 0 to 360 degrees
        // example code doesnt work with TalenFX driveMotor.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter) because we have inbedded encoder in Krakens
        resetEncoders();
      }

      public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();  //ADDED GET VALUE AS DOUBLE BECAUSE OF ODD TYPE FROM DEFAULT //NEEDS TESTING
      }

      public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble();
      }

      public double getTurningPositionRad() {
        return turningMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
      }

      public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
      }

      public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble();
      }

      public double getAbsoluteEncoderRot() {
        return absoluteEncoder.getPosition().getValueAsDouble() * (isAbsoluteEncoderReversed ? -1 : 1); // This means that if boolean returns true then it is assigned the ? #, else :
      }

      public double getAbsoluteEncoderRad() {
          double angleBeforeOffsetIsApplied = absoluteEncoder.getPosition().getValueAsDouble() * 2 * Math.PI; //I Made this part. Not in Video to hopefully convert from rotations into radians.
          double angleAfterOffsetIsApplied = angleBeforeOffsetIsApplied - absoluteEncoderOffsetRAD;
        return angleAfterOffsetIsApplied  * (isAbsoluteEncoderReversed ? -1 : 1); 
      }

      public void resetEncoders() {
         driveMotor.setPosition(0);
         turningMotor.setPosition(getAbsoluteEncoderRot()); //Might have to change to rad. The video used rad but the method says it wants rot sooooo
      }

      public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPositionRad())); //
      }

      public void setDesiredState(SwerveModuleState state) {
       /* if (Math.abs(state.speedMetersPerSecond) < 0.001) {
          stop();
          return; //This immediatly ends the method after the condition is true one time
      } */ //ADD IN THIS TO END THE RESET OF WHEELS WHEN THE JOYSTICK RECIEVES NO INPUT
        state = SwerveModuleState.optimize(state, getState().angle); 
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDController.calculate(getTurningPositionRad(), state.angle.getRadians()) / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //Turning might be in rotations not sure
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
      } //If this is used with the PIDController class's continuous input functionality, the furthest a wheel will ever rotate is 90 degrees. Do we do that?

      public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
      }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
