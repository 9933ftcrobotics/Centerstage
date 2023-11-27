package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(group = "BlueRightWithDistanceAndRoad")

public class BlueRightWithDistanceAndRoad extends LinearOpMode {

    private DcMotor ArmInOut;
    private DcMotor ArmUpDown;




    private Servo LeftClaw;
    private Servo RightClaw;

    private DistanceSensor LeftDistance;
    private DistanceSensor RightDistance;

    int markernumber = 1; //Start with middle spike mark
    @Override
        public void runOpMode() throws InterruptedException {

            ArmInOut = hardwareMap.get(DcMotor.class, "ArmInOut");
            ArmUpDown = hardwareMap.get(DcMotor.class, "ArmUpDown");
            LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
            RightClaw = hardwareMap.get(Servo.class, "RightClaw");

            LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
            RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

            ArmInOut.setDirection(DcMotor.Direction.REVERSE);
            ArmInOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmInOut.setTargetPosition(0);
            ArmInOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmUpDown.setTargetPosition(0);
            ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmUpDown.setPower(0.6);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            waitForStart();

                if (isStopRequested()) return;

                if (LeftDistance.getDistance(DistanceUnit.CM) < 100) {
                    markernumber = 2; //Marker right
                    LeftClaw.setPosition(0.31);
                    //RightClaw.setPosition(0.21);
                    sleep(1000);
                    ArmUpDown.setTargetPosition(100);
                    sleep(1000);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(12)
                            .turn(Math.toRadians(-45))
                            //.forward(8)
                            .build();
                    drive.followTrajectorySequence(Right);

                    ArmUpDown.setTargetPosition(0);
                    sleep(1000);
                    LeftClaw.setPosition(0);
                    sleep(1000);
                    ArmUpDown.setTargetPosition(100);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence RightPartTwo = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .back(5)
                            .turn(Math.toRadians(45))
                            /*.back(12)
                            .turn(Math.toRadians(90))
                            .forward(80)
                            .turn(Math.toRadians(-90))
                            .forward(26.5)
                            .turn(Math.toRadians(90))*/
                            .build();
                    drive.followTrajectorySequence(RightPartTwo);


                            /*ArmUpDown.setTargetPosition(200);
                            ArmInOut.setTargetPosition(80);

                            drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                            TrajectorySequence RightPartThree = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(8)
                            .build();
                            drive.followTrajectorySequence(RightPartThree);
                            RightClaw.setPosition(0);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence RightPartFour = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .back(6)
                            .build();
                    drive.followTrajectorySequence(RightPartFour);*/


                } else if (RightDistance.getDistance(DistanceUnit.CM) < 100) {
                    markernumber = 0; //Marker left

                    LeftClaw.setPosition(0.31);
                    //RightClaw.setPosition(0.21);
                    sleep(1000);
                    ArmUpDown.setTargetPosition(100);
                    sleep(1000);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(20)
                            .turn(Math.toRadians(45))
                            .forward(5)
                            .build();
                    drive.followTrajectorySequence(left);

                    ArmUpDown.setTargetPosition(0);
                    sleep(1000);
                    LeftClaw.setPosition(0);
                    sleep(1000);
                    ArmUpDown.setTargetPosition(100);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence LeftPartTwo = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .back(5)
                            .turn(Math.toRadians(-45))
                            /*.back(12)
                            .turn(Math.toRadians(90))
                            .forward(80)
                            .turn(Math.toRadians(-90))
                            .forward(26.5)
                            .turn(Math.toRadians(90))*/
                            .build();
                    drive.followTrajectorySequence(LeftPartTwo);
                } else {
                    markernumber = 1; //Marker Middle

                    LeftClaw.setPosition(0.3);
                    //RightClaw.setPosition(0.12);
                    sleep(1000);
                    ArmUpDown.setTargetPosition(150);
                    sleep(1000);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence MiddleOne = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(25)
                            .build();
                    drive.followTrajectorySequence(MiddleOne);

                    ArmUpDown.setTargetPosition(0);
                    sleep(1000);
                    LeftClaw.setPosition(0);
                    sleep(1000);
                    ArmUpDown.setTargetPosition(150);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence MiddleTwo = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .back(5)
                            .strafeRight(26)
                            .forward(5)
                            .turn(Math.toRadians(-90))
                            .build();
                    drive.followTrajectorySequence(MiddleTwo);

                    ArmUpDown.setTargetPosition(420);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence MiddleThree = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(10)
                            .build();
                    drive.followTrajectorySequence(MiddleThree);

                    RightClaw.setPosition(-0.1);
                    sleep(1000);

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence MiddleFour = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .back(10)
                            .turn(Math.toRadians(90))
                            .build();
                    drive.followTrajectorySequence(MiddleFour);
                    ArmUpDown.setTargetPosition(0);
                }

            if (markernumber == 0) {
                telemetry.addLine("Left Spike Mark!");
            } else if (markernumber == 1) {
                telemetry.addLine("Middle Spike Mark!");
            } else if (markernumber == 2) {
                telemetry.addLine("Right Spike Mark!");
            }

            telemetry.addData("Right Distance Sensor", RightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance Sensor", LeftDistance.getDistance(DistanceUnit.CM));

    }
}
