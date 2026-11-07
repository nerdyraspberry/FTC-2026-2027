package org.firstinspires.ftc.teamcode.data;

import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumMotor;

public class MecanumConfig {
    public static void apply(MecanumDrive drive) {
        drive
                .setSigns(MecanumMotor.FrontLeft, "-", "-", "+")
                .setSigns(MecanumMotor.FrontRight, "-", "+", "-")
                .setSigns(MecanumMotor.BackLeft, "+", "-", "-")
                .setSigns(MecanumMotor.BackRight, "+", "+", "+");
    }
}
