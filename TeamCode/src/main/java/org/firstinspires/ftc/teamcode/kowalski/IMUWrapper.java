package org.firstinspires.ftc.teamcode.kowalski;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUWrapper {
    private final IMU imu;

    public IMUWrapper(HardwareMap hardwareMap) {
        // de IMU initializeren
        this.imu = hardwareMap.get(IMU.class, "imu");
        // TODO bepaal orientatie en zet de juiste dingen neer bij logo en usb directie
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public double getYaw() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        this.imu.resetYaw();
    }
}
