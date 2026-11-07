package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumMotor;

public class CalibrationTeleOP extends LinearOpMode {
    private IMU imu;
    private MecanumDrive drivetrain;
    private double firstYaw;
    private double secondYaw;
    private double initialLeftSpeed = 1.0;
    private double initialRightSpeed = 1.0;
    private int measuringTime = 1000;
    private double difference;
    private double increase = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        this.initDriveTrain();
        this.initIMU();
        this.resetYaw();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                this.resetYaw();
                firstYaw = this.getYaw();
                drivetrain.setSpeed(initialLeftSpeed, initialRightSpeed);
                sleep(measuringTime);
                secondYaw = this.getYaw();
                drivetrain.setSpeed(0,0);
                difference = firstYaw - secondYaw;
                if (difference <= 5 && difference >= 0) {
                    telemetry.addData("Difference smaller then 5 degrees", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialRightSpeed += 0.01;
                } else if (difference >= 5) {
                    telemetry.addData("Difference smaller then 5 degrees", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialRightSpeed += 0.01;
                } else if (difference > 0) {
                    telemetry.addData("difference", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialRightSpeed += increase;
                }
                else if (difference < 0) {
                    telemetry.addData("difference", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialLeftSpeed += increase;
                }
            }
        }
    }
    private void initIMU() {
        this.imu = hardwareMap.get(IMU.class, "imu");
        // TODO bepaal orientatie en zet de juiste dingen neer bij logo en usb directie
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    // mag alleen gebruikt worden nadat intitIMU()
    private double getYaw() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    // mag alleen gebruikt worden nadat intitIMU()
    private void resetYaw() {
        this.imu.resetYaw();
    }
  private void initDriveTrain() {
      this.drivetrain = new MecanumDrive(
              hardwareMap,
              "left forward", "left backward",
              "right forward", "right backward"
      );
      this.drivetrain
              .setSigns(MecanumMotor.FrontLeft, "-", "-", "+")
              .setSigns(MecanumMotor.FrontRight, "-", "+", "-")
              .setSigns(MecanumMotor.BackLeft, "+", "-", "-")
              .setSigns(MecanumMotor.BackRight, "+", "+", "+");


  }
}