package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.MecanumConfig;
import org.firstinspires.ftc.teamcode.kowalski.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.auto.states.MoveState;

public class BaseAutonomous extends LinearOpMode {
    public MecanumDrive drivetrain;

    public void setAlliance(Alliance alliance) {
        // TODO
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialise hardware
        this.drivetrain = new MecanumDrive(
                hardwareMap,
                "left forward", "left backward",
                "right forward", "right backward"
        );
        MecanumConfig.apply(this.drivetrain);
        // Variables
        StateMachine<BaseAutonomous> stateMachine = new StateMachine<>(this);
        waitForStart();
        // Prime the state machine
        stateMachine.transitionTo(new MoveState());
        while (opModeIsActive()) {
            // Tick the state machine
            stateMachine.tick(getRuntime());
            sleep(20);
        }
        // Ensure every component is fully stalled (otherwise we get a penalty, not good!)
        this.drivetrain.move(0, 0).rotate(0).apply();
    }
}
