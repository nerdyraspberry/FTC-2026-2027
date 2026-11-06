package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.data.Alliance;

@Autonomous(name="Base Autonomous -- Red Alliance", preselectTeleOp="Base TeleOp -- Red Alliance")
public class RedAutonomous extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        this.setAlliance(Alliance.Red);
        super.runOpMode();
    }
}
