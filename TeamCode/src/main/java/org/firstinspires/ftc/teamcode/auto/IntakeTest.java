package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.HardwareUpdate;
import org.firstinspires.ftc.teamcode.commands.PrepareIntake;
import org.firstinspires.ftc.teamcode.commands.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Autonomous
public class IntakeTest extends CommandOpMode {
    IntakeSubsystem intake;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        intake=new IntakeSubsystem(); register(intake);

        //lift=new LiftSubsystem(); register(lift);

        Robot.robotInit(hardwareMap);
        CommandScheduler.getInstance().schedule(new HardwareUpdate());
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new PrepareIntake(intake),
            new WaitUntilCommand(()->IntakeSubsystem.colorDetected!= IntakeSubsystem.COLOR.blank&&IntakeSubsystem.colorDetected!=Robot.enemyColor),
            new Transfer(intake)
        ));
    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
    }
}
