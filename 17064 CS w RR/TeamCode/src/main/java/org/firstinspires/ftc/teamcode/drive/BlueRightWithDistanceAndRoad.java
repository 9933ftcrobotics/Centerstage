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

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            waitForStart();

            if (isStopRequested()) return;

                if (LeftDistance.getDistance(DistanceUnit.CM) < 100) {
                    markernumber = 2; //Marker right

                    LeftClaw.setPosition(0.3);
                    RightClaw.setPosition(0.21);
                    ArmUpDown.setTargetPosition(100);
                    
                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(18)
                            .turn(Math.toRadians(-45))
                            .forward(8)
                            .build();
                            drive.followTrajectorySequence(Right);

                            ArmUpDown.setTargetPosition(0);
                            LeftClaw.setPosition(0);
                            wait(1);
                            ArmUpDown.setTargetPosition(100);

                            drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                            TrajectorySequence RightPartTwo = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .back(8)
                            .turn(Math.toRadians(45))
                            .back(12)
                            .turn(Math.toRadians(90))
                            .forward(80)
                            .turn(Math.toRadians(-90))
                            .forward(26.5)
                            .turn(Math.toRadians(90))
                            .build();
                            drive.followTrajectorySequence(RightPartTwo);

                            ArmUpDown.setTargetPosition(200);
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
                    drive.followTrajectorySequence(RightPartFour);


                } else if (RightDistance.getDistance(DistanceUnit.CM) < 100) {
                    markernumber = 0; //Marker left

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(18)
                            .turn(Math.toRadians(45))
                            .forward(10)
                            .waitSeconds(1)
                            .back(10)
                            .turn(Math.toRadians(-45))
                            .back(18)
                            .turn(Math.toRadians(90))
                            .forward(80)
                            .turn(Math.toRadians(-90))
                            .forward(19.5)
                            .turn(Math.toRadians(90))
                            .forward(8)
                            .waitSeconds(2)
                            .back(8)
                            .turn(Math.toRadians(-90))
                            .back(20)
                            .strafeLeft(18)
                            .waitSeconds(1)
                            .build();
                    drive.followTrajectorySequence(Left);
                } else {
                    markernumber = 1; //Marker Middle

                    drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
                    TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                            .forward(27)
                            .waitSeconds(1)
                            .back(25)
                            .turn(Math.toRadians(90))
                            .forward(80)
                            .turn(Math.toRadians(-90))
                            .forward(23.75)
                            .turn(Math.toRadians(90))
                            .forward(8)
                            .waitSeconds(2)
                            .back(8)
                            .turn(Math.toRadians(-90))
                            .back(24)
                            .strafeLeft(18)
                            .waitSeconds(1)
                            .build();
                    drive.followTrajectorySequence(Middle);
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
