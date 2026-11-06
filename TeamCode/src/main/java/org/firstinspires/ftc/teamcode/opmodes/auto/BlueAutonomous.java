package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.data.Alliance;

@Autonomous(name="Base Autonomous -- Blue Alliance", preselectTeleOp="Base TeleOp -- Blue Alliance")
public class BlueAutonomous extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        this.setAlliance(Alliance.Blue);
        super.runOpMode();
    }
}
