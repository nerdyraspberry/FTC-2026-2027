package org.firstinspires.ftc.teamcode.kowalski.autoshooter;

import org.firstinspires.ftc.teamcode.data.Positionizer;
import org.firstinspires.ftc.teamcode.kowalski.statemachine.State;
import org.firstinspires.ftc.teamcode.opmodes.tele.BaseTeleOp;

public class IdleState extends State<BaseTeleOp> {
    @Override
    public void onTransition(State<BaseTeleOp> from) {
        this.runtime.intake.setPower(Positionizer.OUTTAKE_ON);
        this.runtime.smallOuttake.setPower(Positionizer.OUTTAKE_OFF);
        this.runtime.hatch.setPosition(Positionizer.HATCH_CLOSED);
    }
}
