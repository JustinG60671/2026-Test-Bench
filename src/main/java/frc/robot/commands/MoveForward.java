package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;




public class MoveForward extends SequentialCommandGroup{
    
    public MoveForward() {
        super(
            new ParallelCommandGroup(
                //new InstantCommand(() -> Robot.getIntake().setIntake(IntakeStates.IN), Robot.getIntake()),
                
            )
        );
    }
}