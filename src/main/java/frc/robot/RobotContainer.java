package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveControllerCommand;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {


 private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
 private final XboxController m_driverController = new XboxController(0);


 public RobotContainer() {
   m_swerveSubsystem.setDefaultCommand(new SwerveControllerCommand(
     m_swerveSubsystem,
     () -> -m_driverController.getLeftY(),
     () -> -m_driverController.getLeftX(),
     () -> m_driverController.getRightX(),
     () -> !m_driverController.getYButton())); //WHY IS IT SUPPLIER AND WHY DO I NEED THE ARROW THIGNYS


   configureBindings();
 }
 
 private void configureBindings() {
    new Trigger(m_driverController::getAButton) //CHAT GPT CODE ASK CODY TO EXPLAIN
           .onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
 }


 public Command getAutonomousCommand() {
   return null;
 }
}
