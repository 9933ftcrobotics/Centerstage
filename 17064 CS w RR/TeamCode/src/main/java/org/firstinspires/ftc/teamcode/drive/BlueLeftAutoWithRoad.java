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
@Autonomous(group = "BlueLeftAutoWithRoad")

public class BlueLeftAutoWithRoad extends LinearOpMode {

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
        ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest);
        sleep(500);

        /*if (RightDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Left

            telemetry.addLine("Left Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(14.5, 61, Math.toRadians(-90)));
            TrajectorySequence LeftSpike = drive.trajectorySequenceBuilder(new Pose2d(14.5, 61, Math.toRadians(-90)))

                    .lineToSplineHeading(new Pose2d(15, 35, Math.toRadians(-45)))

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

                    .back(5)
                    .turn(Math.toRadians(-45))
                    .back(12)
                    .lineToSplineHeading(new Pose2d(40, 33, Math.toRadians(0)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(400)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(500)
                    )

                    .forward(7)

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(ArmAndClawPosition.LeftClawOpen)
                    )

                    .back(3)
                    .lineToSplineHeading(new Pose2d(42, 31, Math.toRadians(-90)))
                    .build();

            drive.followTrajectorySequence(LeftSpike);

        //} else if (LeftDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Right

            telemetry.addLine("Right Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(14.5, 61, Math.toRadians(-90)));
            TrajectorySequence RightSpike = drive.trajectorySequenceBuilder(new Pose2d(14.5, 61, Math.toRadians(-90)))

                    .lineToSplineHeading(new Pose2d(14, 37, Math.toRadians(-135)))
                    .forward(4)

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
                    .lineToSplineHeading(new Pose2d(40, 20, Math.toRadians(0)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(400)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(500)
                    )

                    .forward(7)

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(ArmAndClawPosition.LeftClawOpen)
                    )

                    .back(3)
                    .lineToSplineHeading(new Pose2d(42, 31, Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySequence(RightSpike);*/
        //} else {

            //Marker Middle

            telemetry.addLine("Middle Spike Mark!");
            telemetry.update();


            drive.setPoseEstimate(new Pose2d(14.5, 61, Math.toRadians(-90)));
            TrajectorySequence MiddleSpike = drive.trajectorySequenceBuilder(new Pose2d(14.5, 61, Math.toRadians(-90)))
                    .forward(29)

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

                    .back(10)
                    .lineToSplineHeading(new Pose2d(42, 29, Math.toRadians(0)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(400)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(500)
                    )

                    .lineToSplineHeading(new Pose2d(47, 29, Math.toRadians(0)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(ArmAndClawPosition.LeftClawOpen)
                    )

                    .back(4)
                    .lineToSplineHeading(new Pose2d(42, 29, Math.toRadians(0)))
                            .build();

            drive.followTrajectorySequence(MiddleSpike);
        //}
        drive.setPoseEstimate(new Pose2d(40, 28, Math.toRadians(0)));
        TrajectorySequence Final = drive.trajectorySequenceBuilder(new Pose2d(40, 28, Math.toRadians(0)))

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
                //.lineTo(new Vector2d(48, 15))//Park Middle
                .build();

        drive.followTrajectorySequence(Final);


        /*sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }


}