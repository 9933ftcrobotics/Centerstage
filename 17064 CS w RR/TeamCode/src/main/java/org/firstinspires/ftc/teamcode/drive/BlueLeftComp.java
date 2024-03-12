package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ArmAndClawPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "BlueLeft")

public class BlueLeftComp extends LinearOpMode {

    private DcMotor ArmUpDown;
    private DcMotor ArmInOut;

    private Servo LeftClaw;
    private Servo RightClaw;

    private DistanceSensor RightDistance;
    private DistanceSensor LeftDistance;

    @Override
    public void runOpMode() throws InterruptedException {

        ArmUpDown = hardwareMap.get(DcMotor.class, "ArmUpDown");
        ArmInOut = hardwareMap.get(DcMotor.class, "ArmInOut");

        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");

        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");

        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmUpDown.setPower(1);

        ArmInOut.setDirection(DcMotor.Direction.REVERSE);
        ArmInOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmInOut.setTargetPosition(0);
        ArmInOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmInOut.setPower(1);

        RightClaw.setDirection(Servo.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        LeftClaw.setPosition(ArmAndClawPosition.LeftClawClosed);
        RightClaw.setPosition(ArmAndClawPosition.RightClawClosed);
        sleep(1000);
        ArmUpDown.setTargetPosition(100);
        sleep(500);

        drive.setPoseEstimate(new Pose2d(14.5, 61, Math.toRadians(-90)));
        TrajectorySequence Start = drive.trajectorySequenceBuilder(new Pose2d(14.5, 61, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(13.5, 47, Math.toRadians(-90)))
                .build();
        drive.followTrajectorySequence(Start);

        if (LeftDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Left

            telemetry.addLine("Left Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(13.5, 47, Math.toRadians(-90)));
            TrajectorySequence LeftSpike = drive.trajectorySequenceBuilder(new Pose2d(13.5, 47, Math.toRadians(-90)))

                    .lineToSplineHeading(new Pose2d(17, 41, Math.toRadians(-45)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround)
                    )

                    .waitSeconds(0.5 )
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .lineToLinearHeading(new Pose2d(12, 57, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(24, 57, Math.toRadians(-90)))
                    //.lineToSplineHeading(new Pose2d(40, 35, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(430)
                    )

                    //.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(500)
                    )

                    .lineToLinearHeading(new Pose2d(50.5, 35, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(275)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .back(6)
                    //.lineToSplineHeading(new Pose2d(42, 31, Math.toRadians(-90)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .lineToSplineHeading(new Pose2d(42, 58, Math.toRadians(-90)))
                    .lineToSplineHeading(new Pose2d(50, 58, Math.toRadians(-90)))//Park Corner
                    //.lineToSplineHeading(new Pose2d(48, 15, Math.toRadians(-90)))//Park Middle
                    .build();

            drive.followTrajectorySequence(LeftSpike);

            } else if (RightDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Right

            telemetry.addLine("Right Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(13.5, 47, Math.toRadians(-90)));
            TrajectorySequence RightSpike = drive.trajectorySequenceBuilder(new Pose2d(13.5, 47, Math.toRadians(-90)))

                    .lineToSplineHeading(new Pose2d(12, 39, Math.toRadians(-135)))
                    //.forward(3)

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .back(12)
                    //.lineToSplineHeading(new Pose2d(40, 20, Math.toRadians(0)))

                    //.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(430)
                    )

                    //.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(500)
                    )

                    .lineToLinearHeading(new Pose2d(50.5, 22, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(275)
                    )

                    .back(3)
                    //.lineToSplineHeading(new Pose2d(42, 31, Math.toRadians(-90)))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .lineToSplineHeading(new Pose2d(42, 58, Math.toRadians(-90)))
                    .lineToSplineHeading(new Pose2d(50, 58, Math.toRadians(-90)))//Park Corner
                    //.lineToSplineHeading(new Pose2d(48, 15, Math.toRadians(-90)))//Park Middle

                    .build();
            drive.followTrajectorySequence(RightSpike);
        } else {

            //Marker Middle

            telemetry.addLine("Middle Spike Mark!");
            telemetry.update();


            drive.setPoseEstimate(new Pose2d(13.5, 47, Math.toRadians(-90)));
            TrajectorySequence MiddleSpike = drive.trajectorySequenceBuilder(new Pose2d(13.5, 47, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(14.5, 32, Math.toRadians(-90)))

                    /*.addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround)
                    )*/

                    .waitSeconds(0.25)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen)
                    )

                    /*.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )*/

                    .lineToSplineHeading(new Pose2d(14.5, 45, Math.toRadians(-90)))
                    .lineToSplineHeading(new Pose2d(40, 35, Math.toRadians(0)))
                    //.lineToSplineHeading(new Pose2d(42, 29, Math.toRadians(0)))

                    //.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(430)
                    )

                    //.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(500)
                    )

                    .lineToSplineHeading(new Pose2d(50.5, 33, Math.toRadians(0)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(300)
                    )

                    //.back(4)
                    //.lineToSplineHeading(new Pose2d(42, 29, Math.toRadians(0)))

                    //.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    //.waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    //.lineToSplineHeading(new Pose2d(42, 58, Math.toRadians(-90)))
                    .lineToSplineHeading(new Pose2d(43, 33, Math.toRadians(0))) // Back Away From Backdrop
                    .lineToSplineHeading(new Pose2d(48, 59, Math.toRadians(-90)))//Park Corner
                    //.lineToSplineHeading(new Pose2d(48, 15, Math.toRadians(-90)))//Park Middle

                    .build();

            drive.followTrajectorySequence(MiddleSpike);
            }
        }


    }
