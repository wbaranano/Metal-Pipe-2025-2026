package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class liftSetState extends SequentialCommandGroup {
    private final LiftSubsystem.liftState state;
    private final LiftSubsystem lift;
    public liftSetState(LiftSubsystem.liftState state) {
        this.state = state;
        lift = Robot.sys.lift;
    }

    @Override
    public void initialize() {
        super.initialize();

        if (state == lift.state) return;

        // check if we have a custom motion defined
        if (lift.state == LiftSubsystem.liftState.basket && state == LiftSubsystem.liftState.wall) {
            basketToWall().schedule();
        } else if (lift.state == LiftSubsystem.liftState.wall && state == LiftSubsystem.liftState.specimen) {
            wallToSpecimen().schedule();
        } else if (lift.state == LiftSubsystem.liftState.specimen && state == LiftSubsystem.liftState.wall) {
            specimenToWall().schedule();
        } else {
            // reset to intermediary
            SequentialCommandGroup start = null, end = null;
            if (lift.state == LiftSubsystem.liftState.transfer) start = transferToIntermediary();
            else if (lift.state == LiftSubsystem.liftState.basket) start = basketToIntermediary();
            else if (lift.state == LiftSubsystem.liftState.wall) start = wallToIntermediary();
            else if (lift.state == LiftSubsystem.liftState.specimen) start = specimenToIntermediary();

            // go to new state
            if (state == LiftSubsystem.liftState.basket) end = intermediaryToBasket();
            else if (state == LiftSubsystem.liftState.transfer) end = intermediaryToTransfer();
            else if (state == LiftSubsystem.liftState.wall) end = intermediaryToWall();
            else if (state == LiftSubsystem.liftState.specimen) end = intermediaryToSpecimen();

            if (start != null && end != null) new SequentialCommandGroup(start, end).schedule();
        }
    }

    private SequentialCommandGroup wallToSpecimen() {
        return new SequentialCommandGroup(
            new liftTo(LiftSubsystem.constants.tick.specimenDeposit),
            new InstantCommand(() -> {
                LiftSubsystem.liftPreset p = LiftSubsystem.constants.specimenDepositPrepPreset;
                lift.setPivot(p.pivot);
                lift.setPitch(p.pitch);
                lift.setExtension(p.extension);
            }),
            // delay roll so specimen doesnt clip lift bars
            new WaitCommand(300),
            new InstantCommand(() -> lift.apply(LiftSubsystem.constants.specimenDepositPrepPreset)),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.specimen)
        );
    }

    private SequentialCommandGroup specimenToWall() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                lift.apply(LiftSubsystem.constants.specimenCollectionPreset);
                lift.setClawClosed(false); // open claw
            }),
            new liftTo(LiftSubsystem.constants.tick.specimenCollection),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.wall)
        );
    }

    private SequentialCommandGroup basketToWall() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                lift.apply(LiftSubsystem.constants.specimenCollectionPreset);
                lift.setClawClosed(false); // open claw
            }),
            new liftTo(LiftSubsystem.constants.tick.specimenCollection),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.wall)
        );
    }

    private SequentialCommandGroup transferToIntermediary() {
        return new SequentialCommandGroup(
            // close claw if need be
            new ConditionalCommand(
                new InstantCommand(),
                new SequentialCommandGroup(
                    new InstantCommand(lift::toggleClaw),
                    new WaitCommand(500)
                ),
                lift::isClawClosed
            ),
            new liftTo(LiftSubsystem.constants.tick.intermediary),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.intermediary)
        );
    }

    private SequentialCommandGroup basketToIntermediary() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> Robot.sys.lift.apply(LiftSubsystem.constants.transferPickupPreset)),
            new liftTo(LiftSubsystem.constants.tick.intermediary),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.intermediary)
        );
    }

    private SequentialCommandGroup wallToIntermediary() {
        return new SequentialCommandGroup(
            new liftTo(LiftSubsystem.constants.tick.intermediary),
            new InstantCommand(() -> lift.apply(LiftSubsystem.constants.transferPickupPreset)),
            new WaitCommand(1500),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.intermediary)
        );
    }

    private SequentialCommandGroup specimenToIntermediary() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> lift.apply(LiftSubsystem.constants.transferPickupPreset)),
            new liftTo(LiftSubsystem.constants.tick.intermediary),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.intermediary)
        );
    }

    private SequentialCommandGroup intermediaryToBasket() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new liftTo(LiftSubsystem.constants.tick.highBasket),
                new SequentialCommandGroup(
                    new WaitCommand(250),
                    new InstantCommand(() -> lift.apply(LiftSubsystem.constants.basketPreset))
                )
            ),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.basket)
        );
    }

    private SequentialCommandGroup intermediaryToTransfer() {
        return new SequentialCommandGroup(
            // open claw if need be
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(lift::toggleClaw),
                    new WaitCommand(500)
                ),
                new InstantCommand(),
                lift::isClawClosed
            ),
            new liftTo(LiftSubsystem.constants.tick.transfer),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.transfer)
        );
    }

    private SequentialCommandGroup intermediaryToWall() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> lift.apply(LiftSubsystem.constants.specimenCollectionPreset)),
            new WaitCommand(1000),
            new liftTo(LiftSubsystem.constants.tick.specimenCollection),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.wall)
        );
    }

    private SequentialCommandGroup intermediaryToSpecimen() {
        return new SequentialCommandGroup(
            new liftTo(LiftSubsystem.constants.tick.specimenDeposit),
            new InstantCommand(() -> lift.apply(LiftSubsystem.constants.specimenDepositPrepPreset)),
            new InstantCommand(() -> lift.state = LiftSubsystem.liftState.specimen)
        );
    }
}
