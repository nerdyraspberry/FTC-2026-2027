package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.data.MecanumConfig;
import org.firstinspires.ftc.teamcode.kowalski.IMUWrapper;
import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumMotor;

public class CalibrationTeleOP extends LinearOpMode {
    private IMUWrapper imu;
    private MecanumDrive drivetrain;
    private double firstYaw;
    private double secondYaw;
    private double initialLeftSpeed = 1.0;
    private double initialRightSpeed = 1.0;
    private int measuringTime = 1000;
    private double difference;
    private double increase = 0.05;
    private double precisionIncrease = 0.01;
    private boolean firsttime = true;
    private boolean leaningRight = false;
    // runt als je op x drukt, displayed een speed correctie die het maakt, op basis van het verschil in hoek na een stukje rijden, herhaal dit vaak, totdat het recht rijd en kopieer dan deze waarden voor de speed.

    @Override
    public void runOpMode() throws InterruptedException {
        this.initDriveTrain();

        this.imu = new IMUWrapper(hardwareMap);
        this.imu.resetYaw();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                this.imu.resetYaw();
                firstYaw = this.imu.getYaw();
                drivetrain.setDrivingCorrection(initialLeftSpeed, initialRightSpeed);
                drivetrain.move(0, 1).apply();
                sleep(measuringTime);
                secondYaw = this.imu.getYaw();
                drivetrain.move(0, 0).apply();
                difference = firstYaw - secondYaw;
                if (firsttime && difference >0) {
                    leaningRight = true;
                    firsttime = false;
                }
                if (difference <= 5 && difference >= 0 && leaningRight) {
                    telemetry.addData("Difference smaller then 5 degrees", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialLeftSpeed -= precisionIncrease;
                } else if (difference >= -5 && difference <0 && leaningRight) {
                    telemetry.addData("Difference smaller then 5 degrees", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialLeftSpeed += precisionIncrease;
                } else if (difference > 0 && leaningRight) {
                    telemetry.addData("difference", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialLeftSpeed -= increase;
                }
                else if (difference < 0 && leaningRight) {
                    telemetry.addData("difference", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialLeftSpeed += increase;
                }
                else if (difference <= 5 && difference >= 0) {
                    telemetry.addData("Difference smaller then 5 degrees", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialRightSpeed += precisionIncrease;
                } else if (difference >= -5 && difference <0) {
                    telemetry.addData("Difference smaller then 5 degrees", difference);
                    telemetry.addData("RightSpeed", initialRightSpeed);
                    telemetry.addData("LeftSpeed", initialLeftSpeed);
                    telemetry.update();
                    initialRightSpeed -= precisionIncrease;
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
                    initialRightSpeed -= increase;
                }
            }
        }
    }

  private void initDriveTrain() {
      this.drivetrain = new MecanumDrive(
              hardwareMap,
              "left forward", "left backward",
              "right forward", "right backward"
      );
      MecanumConfig.apply(this.drivetrain);
  }
}