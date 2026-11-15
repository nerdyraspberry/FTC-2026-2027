package org.firstinspires.ftc.teamcode.kowalski.autoshooter;

import org.firstinspires.ftc.teamcode.data.Positionizer;
import org.firstinspires.ftc.teamcode.kowalski.statemachine.State;
import org.firstinspires.ftc.teamcode.opmodes.tele.BaseTeleOp;

public class SpinUpState extends State<BaseTeleOp> {
    private boolean automatic = true;

    public SpinUpState(boolean automatic) {
        this.automatic = automatic;
    }

    @Override
    public void onTransition(State<BaseTeleOp> from) {
        this.runtime.smallOuttake.setPower(Positionizer.OUTTAKE_ON);
        this.runtime.hatch.setPosition(Positionizer.HATCH_CLOSED);
    }

    @Override
    public void onTick(double timeSinceStart) {
        if (this.automatic && timeSinceStart >= Positionizer.SPIN_UP_TIME) {
            this.stateMachine.transitionTo(new ShootingState(this.automatic));
        }
    }
}
