package org.firstinspires.ftc.teamcode.kowalski.autoshooter;

import org.firstinspires.ftc.teamcode.data.Positionizer;
import org.firstinspires.ftc.teamcode.kowalski.statemachine.State;
import org.firstinspires.ftc.teamcode.opmodes.tele.BaseTeleOp;

public class IdleState extends State<BaseTeleOp> {
    @Override
    public void onTransition(State<BaseTeleOp> from) {
        this.runtime.smallOuttake.setPower(Positionizer.OUTTAKE_OFF);
        this.runtime.hatch.setPosition(Positionizer.HATCH_CLOSED);
        this.runtime.middle.setPower(Positionizer.MIDDLE_OFF);
    }
}
