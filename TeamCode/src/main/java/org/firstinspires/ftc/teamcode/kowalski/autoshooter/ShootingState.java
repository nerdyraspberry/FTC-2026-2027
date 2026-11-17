package org.firstinspires.ftc.teamcode.kowalski.autoshooter;

import org.firstinspires.ftc.teamcode.data.Positionizer;
import org.firstinspires.ftc.teamcode.kowalski.statemachine.State;
import org.firstinspires.ftc.teamcode.opmodes.tele.BaseTeleOp;

public class ShootingState extends State<BaseTeleOp> {
    private boolean automatic = true;

    public ShootingState(boolean automatic) {
        this.automatic = automatic;
    }

    @Override
    public void onTransition(State<BaseTeleOp> from) {
        this.runtime.smallOuttake.setPower(Positionizer.OUTTAKE_ON);
        this.runtime.hatch.setPosition(Positionizer.HATCH_OPEN);
        this.runtime.middle.setPower(Positionizer.MIDDLE_ON);
    }

    @Override
    public void onTick(double timeSinceStart) {
        if (this.automatic && timeSinceStart >= Positionizer.SHOOT_TIME) {
            this.stateMachine.transitionTo(new IdleState());
        }
    }
}
