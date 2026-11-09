package org.firstinspires.ftc.teamcode.opmodes.auto.states;

import org.firstinspires.ftc.teamcode.kowalski.statemachine.State;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous;

public class MoveState extends State<BaseAutonomous> {
    @Override
    public void onTransition(State<BaseAutonomous> from) {
        this.runtime.drivetrain.move(0, 1).apply();
    }

    @Override
    public void onTick(double timeSinceStart) {
        if (timeSinceStart >= 3.0)
            this.stateMachine.transitionTo(new StallState());
    }
}
