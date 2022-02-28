// package frc.robot.subsystems;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.RobotContainer;
// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import com.revrobotics.*;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// public class IntakeSubsystem extends SubsystemBase {
//     DigitalInput toplimitSwitch = new DigitalInput(0);
//     CANSparkMax converyerMotorOne = new CANSparkMax(IntakeConstants.CONVEYER_MOTOR_ONE, MotorType.kBrushless);
//     CANSparkMax conveyerTwo = new CANSparkMax(IntakeConstants.CONVERYER_TWO, MotorType.kBrushless);
//     CANSparkMax intake = new CANSparkMax(IntakeConstants.INTAKE, MotorType.kBrushless);


//     public IntakeSubsystem(){
    

//         converyerMotorOne.setIdleMode(IdleMode.kCoast);
//         conveyerTwo.setIdleMode(IdleMode.kCoast);
//         intake.setIdleMode(IdleMode.kCoast);
//     }


 
    
    

//     public void spitOutBall(double SPIT_OUT_BALL){
// converyerMotorOne.set(SPIT_OUT_BALL);
// conveyerTwo.set(SPIT_OUT_BALL);
// intake.set(SPIT_OUT_BALL);
//     }


//     public void TakeInBall(double Intakespeed) {
//         if (Intakespeed > 0){
//             if(toplimitSwitch.get()){
//                 converyerMotorOne.set(0);
//                 conveyerTwo.set(0);
//                 intake.set(0);
//             }
//             else{
//                 converyerMotorOne.set(Intakespeed);
//                 conveyerTwo.set(Intakespeed);
//                 intake.set(Intakespeed);
//           }
//     }
//     }
// }
