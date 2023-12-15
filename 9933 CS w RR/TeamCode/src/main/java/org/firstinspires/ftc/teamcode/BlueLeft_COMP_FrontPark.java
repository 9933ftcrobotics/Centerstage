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


public class BlueLeft_COMP_FrontPark extends LinearOpMode {
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
        drive.setPoseEstimate(new Pose2d(14, 66, Math.toRadians(-90)));

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(14, 66, Math.toRadians(-90)))
                .addTemporalMarker(() ->
                        RightClaw.setPosition(0.5)
                )
                .addTemporalMarker(() ->
                        LeftClaw.setPosition(0.5)
                )
                .waitSeconds(1)
                .addTemporalMarker(() ->
                        ArmUpDown.setTargetPosition(100)
                )
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(12,52.25,Math.toRadians(-90)))
                .waitSeconds(1)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(12,52.25, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(12, 37, Math.toRadians(-90)))
                .addTemporalMarker(() ->
                        LeftClaw.setPosition(0.3)
                )
                .lineToLinearHeading(new Pose2d(42, 37, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() ->
                        ArmUpDown.setTargetPosition(1600)
                )
                .waitSeconds(2.5)
                .lineToLinearHeading(new Pose2d(47, 42.5, Math.toRadians(180)))
                .waitSeconds(2)
                .addTemporalMarker(() ->
                        RightClaw.setPosition(0.2)
                )
                .waitSeconds(2.5)
                .addTemporalMarker(() ->
                        ArmUpDown.setTargetPosition(50)
                )
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50, 18, Math.toRadians(-90)))
                .build();

        drive.followTrajectorySequence(traj1);

        if(RightDistance.getDistance(DistanceUnit.CM) < 50)
        {
            Spike = "Right Spike";
            traj2 = drive.trajectorySequenceBuilder(new Pose2d(12,45, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(12, 37, Math.toRadians(180)))
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0.3)
                    )
                    .lineToLinearHeading(new Pose2d(38, 32, Math.toRadians(180)))
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(1600)
                    )
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(47, 31, Math.toRadians(180)))
                    .waitSeconds(1)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(0.2)
                    )
                    .waitSeconds(2.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(50)
                    )
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(45, 18, Math.toRadians(-90)))
                    .build();
        }
        else if(LeftDistance.getDistance(DistanceUnit.CM) < 50)
        {
            Spike = "Left Spike";
            traj2 = drive.trajectorySequenceBuilder(new Pose2d(12,49.25, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(24, 40, Math.toRadians(-90)))
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0.3)
                    )
                    .waitSeconds(2)

                    .lineToLinearHeading(new Pose2d(38, 42.5, Math.toRadians(180)))
                    .waitSeconds(1)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(1600)
                    )
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(47, 42.5, Math.toRadians(180)))
                    .waitSeconds(1)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(0.2)
                    )
                    .waitSeconds(2.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(50)
                    )
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(50, 18, Math.toRadians(-90)))
                    .build();
            /*traj1 = drive.trajectorySequenceBuilder(new Pose2d(12, 66, Math.toRadians(-90)))
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(0.5)
                    )
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(0.5)
                    )
                    .waitSeconds(1)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(100)
                    )
                    .waitSeconds(1)

                    .lineToLinearHeading(new Pose2d(28, 32, Math.toRadians(180)))
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(0)
                    )
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(0)
                    )
                    .waitSeconds(1)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(200)
                    )
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(30, 35, Math.toRadians(180)))
                    .build();*/
        }
        else
        {
            Spike = "Center Spike";

        }
        telemetry.addData("Left Distance", LeftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", RightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Spike Selected", Spike);
        telemetry.update();



              //  .strafeLeft(25)
              //  .back(15)
                //.forward(20)
                //.lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                // .lineToSplineHeading(new Pose2d(8, -36, Math.toRadians(180)))
                //.forward(55)
                //.back(55)
                // .lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                // .back(5)
                // .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
               // .build();
        /*TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(45, 35, Math.toRadians(180)))
                .strafeLeft(25)
                .back(15)








                .build();*/




        drive.followTrajectorySequence(traj2);
        /*sleep(200);
        ArmUpDown.setTargetPosition(1500);
        sleep(2000);
        LeftClaw.setPosition(0.3);
        sleep(1000);
        ArmUpDown.setTargetPosition(0);
        sleep(800);
        drive.followTrajectorySequence(traj3);*/

    }
}
