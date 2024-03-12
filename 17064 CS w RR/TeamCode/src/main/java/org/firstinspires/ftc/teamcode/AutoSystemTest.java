package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.apache.commons.math3.analysis.function.Asin;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ArmAndClawPosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ArmAndClawPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


import java.lang.Math;

@TeleOp(name = "AutoSystemTest", group = "")
//@Disabled
public class AutoSystemTest extends LinearOpMode {

    private DcMotor ClimberRight;
    private DcMotor ClimberLeft;
    private DcMotor ArmInOut;
    private DcMotor ArmUpDown;


    private Servo LeftClaw;
    private Servo RightClaw;
    private Servo DroneLauncher;

    double RightClawPositionClosed = 0.21;
    double LeftClawPositionClosed = 0.27;

    private boolean BisPressed = false;

    private boolean XisPressed = false;

    private boolean RunTest = false;

    int stepNumber = 0;

    @Override
    public void runOpMode () {
        ClimberLeft = hardwareMap.get(DcMotor.class, "ClimberLeft");
        ClimberRight = hardwareMap.get(DcMotor.class, "ClimberRight");
        ArmInOut = hardwareMap.get(DcMotor.class, "ArmInOut");
        ArmUpDown = hardwareMap.get(DcMotor.class, "ArmUpDown");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");

        DroneLauncher.setDirection(Servo.Direction.REVERSE);


        ClimberLeft.setDirection(DcMotor.Direction.REVERSE);
        ClimberLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClimberLeft.setTargetPosition(0);
        ClimberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //  ClimberRight.setDirection(DcMotor.Direction.REVERSE);
        ClimberRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClimberRight.setTargetPosition(0);
        ClimberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        ArmInOut.setDirection(DcMotor.Direction.REVERSE);
        ArmInOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmInOut.setTargetPosition(0);
        ArmInOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightClaw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {


            ArmUpDown.setPower(0.5);
            ArmInOut.setPower(1);
            ClimberLeft.setPower(1);
            ClimberRight.setPower(1);
            while (opModeIsActive()) {
                telemetry.addLine("Press B to start test. Press X to stop test.");
                telemetry.update();

                if (gamepad1.b | gamepad2.b){
                    stepNumber = 10;
                }

                if (gamepad1.x | gamepad2.x){
                    stepNumber = 0;
                }

                switch (stepNumber) {
                    case 10:
                        telemetry.addLine("Testing Servos Part 1");
                        sleep(1000);
                        LeftClaw.setPosition(0.3);
                        RightClaw.setPosition(0.3);
                        stepNumber = 20;
                        break;
                    case 20:
                        telemetry.addLine("Testing Servos Part 2");
                        sleep(1000);
                        stepNumber = 30;
                        break;
                    case 30:
                        telemetry.addLine("Testing Servos Part 3");
                        LeftClaw.setPosition(0);
                        RightClaw.setPosition(0);
                        sleep(1000);
                        stepNumber = 40;
                        break;
                    case 40:
                        telemetry.addLine("Testing Climbing Arms Part 1");
                        sleep(1000);
                        ClimberLeft.setTargetPosition(4900);
                        ClimberRight.setTargetPosition(4900);
                        stepNumber = 50;
                        break;
                    case 50:
                        telemetry.addLine("Testing Climbing Arms Part 2");
                        sleep(3000);
                        stepNumber = 60;
                        break;
                    case 60:
                        telemetry.addLine("Testing Climbing Arms Part 3");
                        ClimberLeft.setTargetPosition(0);
                        ClimberRight.setTargetPosition(0);
                        sleep(3000);
                        stepNumber = 70;
                        break;
                    case 70:
                        telemetry.addLine("Testing Main Arm Up Down Part 1. Arm Will Raise To The Highest Position!");
                        sleep(1000);
                        ArmUpDown.setTargetPosition(590);
                        /*sleep(1500);
                        ArmUpDown.setTargetPosition(100);
                        sleep(1000);*/
                        stepNumber = 80;
                        break;
                    case 80:
                        telemetry.addLine("Testing Main Arm Up Down Part 2");
                        sleep(1500);
                        stepNumber = 90;
                        break;
                    case 90:
                        telemetry.addLine("Testing Main Arm Up Down Part 3");
                        ArmUpDown.setTargetPosition(150);
                        sleep(1500);
                        stepNumber = 100;
                        break;
                    case 100:
                        telemetry.addLine("Testing Main Arm In Out Part 1. Arm Will Raise to Highest And Farthest Position!");
                        sleep(1000);
                        ArmUpDown.setTargetPosition(590);
                        sleep(1000);
                        ArmInOut.setTargetPosition(1590);
                        stepNumber = 110;
                        break;
                    case 110:
                        telemetry.addLine("Testing Main Arm In Out Part 2");
                        sleep(1000);
                        ArmInOut.setTargetPosition(0);
                        stepNumber = 120;
                        break;
                    case 120:
                        telemetry.addLine("Testing Main Arm In Out Part 3");
                        sleep(750);
                        ArmUpDown.setTargetPosition(150);
                        stepNumber = 130;
                        break;
                    case 130:
                        telemetry.addLine("Testing Drone Launcher Part 1");
                        sleep(1000);
                        DroneLauncher.setPosition(1);
                        stepNumber = 140;
                        break;
                    case 140:
                        telemetry.addLine("Testing Drone Launcher Part 2");
                        sleep(1000);
                        stepNumber = 150;
                        break;
                    case 150:
                        telemetry.addLine("Testing Drone Launcher Part 3");
                        DroneLauncher.setPosition(0);
                        stepNumber = 160;
                        break;
                    case 160:
                        telemetry.addLine("Test is done! Test will restart soon. Press X anytime to stop test.");
                        sleep(1000);
                        stepNumber = 10;
                        break;
                    default:
                        telemetry.addLine("Invalid size number");
                }

                telemetry.update();
            }

        }
    }

}