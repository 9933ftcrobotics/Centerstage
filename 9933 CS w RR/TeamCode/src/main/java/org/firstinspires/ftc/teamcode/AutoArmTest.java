package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Drive")


public class AutoArmTest extends LinearOpMode {
    //@Override

    private Servo RightClaw;

    private Servo LeftClaw;

    private DcMotor ArmUpDown;

    private DistanceSensor LeftDistance;
    private DistanceSensor RightDistance;
    String Spike;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RightClaw=hardwareMap.servo.get("RightClaw");
        LeftClaw=hardwareMap.servo.get("LeftClaw");

        RightClaw.setDirection(Servo.Direction.REVERSE);

        ArmUpDown = hardwareMap.get(DcMotor.class, "ArmUpDown");

        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        ArmUpDown.setPower(0.5);

        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-38, -66, Math.toRadians(90)));

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-38, -66, Math.toRadians(90)))
                .addTemporalMarker(() ->
                        RightClaw.setPosition(0)
                )
                .addTemporalMarker(() ->
                        LeftClaw.setPosition(0.5)
                )
                .waitSeconds(1)
                .addTemporalMarker(() ->
                        ArmUpDown.setTargetPosition(1675)
                )
                .waitSeconds(1.5)
                .back(10)
                .addTemporalMarker(() ->
                        LeftClaw.setPosition(0.4)
                )
                .waitSeconds(1)
                .back(3)
                .addTemporalMarker(() ->
                        LeftClaw.setPosition(0)
                )

                .forward(5)
                .addTemporalMarker(() ->
                        ArmUpDown.setTargetPosition(100)
                )
                .forward(5)
                //.lineToLinearHeading(new Pose2d(-35.25,-52.25,Math.toRadians(90)))
                .waitSeconds(5)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(-35.25,-52.25, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-35.25, -37, Math.toRadians(90)))
                .addTemporalMarker(() ->
                        LeftClaw.setPosition(0.3)
                )
                .waitSeconds(2)
                .build();

        drive.followTrajectorySequence(traj1);



    }
}
