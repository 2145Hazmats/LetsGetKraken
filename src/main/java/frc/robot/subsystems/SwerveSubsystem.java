package frc.robot.subsystems;


import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

//import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
       Constants.DriveConstants.kFrontLeftDriveMotorPort,
       Constants.DriveConstants.kFrontLeftTurningMotorPort,
       Constants.DriveConstants.kFrontLeftDriveMotorReversed,
       Constants.DriveConstants.kFrontLeftTurningMotorReversed,
       Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
       Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);


   private final SwerveModule frontRight = new SwerveModule(
       Constants.DriveConstants.kFrontRightDriveMotorPort,
       Constants.DriveConstants.kFrontRightTurningMotorPort,
       Constants.DriveConstants.kFrontRightDriveMotorReversed,
       Constants.DriveConstants.kFrontRightTurningMotorReversed,
       Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
       Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);

   private final SwerveModule backLeft = new SwerveModule(
       Constants.DriveConstants.kBackLeftDriveMotorPort,
       Constants.DriveConstants.kBackLeftTurningMotorPort,
       Constants.DriveConstants.kBackLeftDriveMotorReversed,
       Constants.DriveConstants.kBackLeftTurningMotorReversed,
       Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
       Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);

   private final SwerveModule backRight = new SwerveModule(
       Constants.DriveConstants.kBackRightDriveMotorPort,
       Constants.DriveConstants.kBackRightTurningMotorPort,
       Constants.DriveConstants.kBackRightDriveMotorReversed,
       Constants.DriveConstants.kBackRightTurningMotorReversed,
       Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort,
       Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);

  private StructArrayPublisher<SwerveModuleState> statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("swerveStates", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("desiredSwerveStates", SwerveModuleState.struct).publish();

  //private Pigeon2 gyro = new Pigeon2(Constants.DriveConstants.kPigeonGyroPort);
  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  public SwerveSubsystem() {
      new Thread(() -> {
           try {
               // Sleep for 1000 milliseconds (1 second)
               Thread.sleep(1000);
               // Call the zeroHeading method after the sleep
               zeroHeading();
           } catch (InterruptedException e) {
               // Handle the exception if the thread is interrupted
               System.out.println("Bruh, The Gyro Didn't Zero Before the Command Got interupped");
           }
       }).start(); //THE ABOVE CODE IS MEANT TO WAIT 1 SECOND BEFORE RESETING GYRO TO ALLOW TIME FOR IT TO CALIBRATE AND STUFF
     }

     public void zeroHeading() {
       gyro.reset();
     }

     public double getHeading() {
        //return Math.IEEEremainder(-gyro.getAngle(), 360);  //NOT EXACTLY SURE I THINK IT WILL LOOP 360 BACK AROUND NEEDS TESTING
        return gyro.getAngle();
      }


     public Rotation2d getRotation2d() {
      //return new Rotation2d(Units.degreesToRadians(getHeading()));
      return Rotation2d.fromDegrees(getHeading());
     }

 @Override
 public void periodic() {
   // This method will be called once per scheduler run
   SmartDashboard.putNumber("Robot Heading", getHeading());
   SmartDashboard.putNumber("Gyro Output", gyro.getAngle());
   SmartDashboard.putNumber("frontLeftMod Velocity", frontLeft.getDriveVelocity()); //Units are probably wrong here not sure
   SmartDashboard.putNumber("AverageMods Velocity", (frontLeft.getDriveVelocity() + frontRight.getDriveVelocity() + backLeft.getDriveVelocity() + backRight.getDriveVelocity())/4);

   SmartDashboard.putNumber("FrontLeftRot", frontLeft.getAbsoluteEncoderRot()); //Units are probably wrong here not sure
   SmartDashboard.putNumber("FrontRightRot", frontRight.getAbsoluteEncoderRot());
   SmartDashboard.putNumber("BackLeftRot", backLeft.getAbsoluteEncoderRot());
   SmartDashboard.putNumber("BackRightRot", backRight.getAbsoluteEncoderRot());

   statePublisher.set(new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
  }


 public void stopModules() {
   frontLeft.stop();
   frontRight.stop();
   backLeft.stop();
   backRight.stop();
 }


 public void setModuleStates(SwerveModuleState[] desiredStates) {
   desiredStatePublisher.set(new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()});
   SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
   frontLeft.setDesiredState(desiredStates[0]);
   frontRight.setDesiredState(desiredStates[1]);
   backLeft.setDesiredState(desiredStates[2]);
   backRight.setDesiredState(desiredStates[3]);
 }

}
