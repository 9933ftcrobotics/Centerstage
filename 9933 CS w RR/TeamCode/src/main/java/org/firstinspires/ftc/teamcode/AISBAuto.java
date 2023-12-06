package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Drive")


public class AISBAuto extends LinearOpMode {
    //@Override

    private Servo RightClaw;

    private Servo LeftClaw;

    private DcMotor ArmUpDown;
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RightClaw=hardwareMap.servo.get("RightClaw");
        LeftClaw=hardwareMap.servo.get("LeftClaw");
        ArmUpDown = hardwareMap.get(DcMotor.class, "ArmUpDown");

        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(100);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        RightClaw.setDirection(Servo.Direction.REVERSE);
        LeftClaw.setPosition(0.5);
        RightClaw.setPosition(0.5);
        sleep(3000);
        ArmUpDown.setPower(0.5);

        RightClaw.setDirection(Servo.Direction.REVERSE);
        LeftClaw.setPosition(0.5);
        RightClaw.setPosition(0.5);
        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(14, -66, Math.toRadians(90)));
        TrajectorySequence Center = drive.trajectorySequenceBuilder(new Pose2d(14, -66, Math.toRadians(90)))

                .lineToLinearHeading(new Pose2d(16.5,-38,Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    LeftClaw.setPosition(0);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(35,-37,Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    ArmUpDown.setTargetPosition(1400);
                })

                //.lineToLinearHeading(new Pose2d(37.5,-37,Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    //RightClaw.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(44.5,-37,Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    RightClaw.setPosition(0);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    ArmUpDown.setTargetPosition(90);
                })
                //End of Yellow



                //Go to other side
                .lineToLinearHeading(new Pose2d(30,-37,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-30,-37,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-60,-44,Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    RightClaw.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-30,-37,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(35,-37,Math.toRadians(180)))



                //Back at it again at krispy kreme
                .addTemporalMarker(() -> {
                    ArmUpDown.setTargetPosition(1400);
                })

                //.lineToLinearHeading(new Pose2d(37.5,-37,Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    //RightClaw.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(44.5,-37,Math.toRadians(180)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    RightClaw.setPosition(0);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    ArmUpDown.setTargetPosition(100);
                })
                .lineToLinearHeading(new Pose2d(40,-25,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52,-13,Math.toRadians(90)))
                //End of Auto

                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(14,-66,Math.toRadians(90)))
                .build();


        drive.followTrajectorySequence(Center);

    }
}
