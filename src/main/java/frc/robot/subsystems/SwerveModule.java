package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;

  private final CANcoder driveEncoder;
  private final CANcoder turningEncoder;

  private final PIDController turningPIDController; //or you could use onboard! Pretty sure we can run 2 channels of PID on Krakens

  private final AnalogInput absoluteEncoder;  //plugged into robo rio
  private final boolean isAbsoluteEncoderReversed;
  private final double absoluteEncoderOffsetRAD;

  public SwerveModule(int driveMotorID, int turningMotorID, boolean isDriveMotorReversed, boolean isTurningMotorReversed,
          int absoluteEncoderID, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed) {
        
        absoluteEncoderOffsetRAD = absoluteEncoderOffset; 
          //why does tutorial do this.absoluteEncoderOffsetRAD = absoluteEncoderOffset'?
          // why does tutorial say  this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);
        
        driveMotor = new TalonFX(driveMotorID);
        turningMotor = new TalonFX(turningMotorID);

        driveMotor.setInverted(isDriveMotorReversed);
        turningMotor.setInverted(isTurningMotorReversed);

        turningPIDController = new PIDController(Constants.SwerveModConstants.kturingMotorP,Constants.SwerveModConstants.kturingMotorI,Constants.SwerveModConstants.kturingMotorD);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);  //Im assuming that -180 degrees is 360 degrees away from 180. Why is not 0 to 360 degrees
        // example code doesnt work with TalenFX driveMotor.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter) because we have inbedded encoder in Krakens
 
      }

      public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();  //ADDED GET VALUE AS DOUBLE BECAUSE OF ODD TYPE FROM DEFAULT //NEEDS TESTING
      }

      public double getTurningPosition() {
        return turningMotor.getPosition().getValueAsDouble();
      }

      public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
      }

      public double getTurningVelocity() {
        return turningMotor.getVelocity().getValueAsDouble();
      }

      public double getAbsoluteEncoderRad() {
        return 
      }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
