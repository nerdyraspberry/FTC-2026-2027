package org.firstinspires.ftc.teamcode.opmodes.auto.states;

import org.firstinspires.ftc.teamcode.kowalski.statemachine.State;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous;

public class StallState extends State<BaseAutonomous> {
    @Override
    public void onTransition(State<BaseAutonomous> from) {
        this.runtime.drivetrain.move(0, 0).apply();
    }
}
