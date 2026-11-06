package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.data.Alliance;

@TeleOp(name="Base TeleOp -- Blue Alliance")
public class BlueTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        this.setAlliance(Alliance.Blue);
        super.runOpMode();
    }
}
