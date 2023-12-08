package org.firstinspires.ftc.teamcode.drive;

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



/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "RedRightAutoWithRoad")
@Disabled
public class RedRightAutoWithRoad extends LinearOpMode {

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

        if (RightDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Left

            telemetry.addLine("Left Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(14.5, -61, Math.toRadians(90)));
            TrajectorySequence LeftSpike = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61, Math.toRadians(90)))

                    .lineToSplineHeading(new Pose2d(8,-35,Math.toRadians(135))) // To Left Spike Mark Line

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
                    .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(0))) // To BackDrop

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(400)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(400)
                    )

                    .lineToSplineHeading(new Pose2d(46, -30, Math.toRadians(0))) // On Backdrop

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(ArmAndClawPosition.LeftClawOpen)
                    )

                    .lineTo(new Vector2d(40, -30))
                    .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(90)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .lineTo(new Vector2d(48, -59)) // Park Low
                    //.lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90))) //Park Middle

                    .build();

            drive.followTrajectorySequence(LeftSpike);

        } else if (LeftDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Right

            telemetry.addLine("Left Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(14.5, -61, Math.toRadians(90)));
            TrajectorySequence RightSpike = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61, Math.toRadians(90)))

                    .lineToSplineHeading(new Pose2d(15, -30, Math.toRadians(45)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(0)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .lineToSplineHeading(new Pose2d(12, -55, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(40, -42, Math.toRadians(0)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(400)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(400)
                    )

                    .lineTo(new Vector2d(45, -42))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0)
                    )

                    .lineToSplineHeading(new Pose2d(34, -45, Math.toRadians(90)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .lineTo(new Vector2d(50, -59))//Park Low
                    //.lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90)))//Park Middle

                    .build();
            drive.followTrajectorySequence(RightSpike);
        } else {

            //Marker Middle

            telemetry.addLine("Middle Spike Mark!");
            telemetry.update();


            drive.setPoseEstimate(new Pose2d(14.5, -61, Math.toRadians(90)));
            TrajectorySequence MiddleSpike = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61, Math.toRadians(90)))

                    .lineTo(new Vector2d(14.5, -34))

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

                    .lineTo(new Vector2d(14.5, -45))
                    .lineToSplineHeading(new Pose2d(40, -35, Math.toRadians(0)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(400)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(400)
                    )

                    .lineTo(new Vector2d(45, -35))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0)
                    )

                    .lineToSplineHeading(new Pose2d(34, -45, Math.toRadians(90)))

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .lineTo(new Vector2d(50, -59)) //Park Low
                    //.lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90))) //Park Middle
                    .build();
            drive.followTrajectorySequence(MiddleSpike);
        }

    }

}