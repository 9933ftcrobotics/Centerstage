package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Drive")


public class DemoAuto extends LinearOpMode {
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
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmUpDown.setPower(0.5);
        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(12, 66, Math.toRadians(-90)));
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(12, 66, Math.toRadians(-90)))
                .strafeTo(new Vector2d(24,35))
                //.forward(20)
                //.lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                // .lineToSplineHeading(new Pose2d(8, -36, Math.toRadians(180)))
                //.forward(55)
                //.back(55)
                // .lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                // .back(5)
                // .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(24, 35, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(48, 35, Math.toRadians(180)))
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
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(45, 35, Math.toRadians(180)))
                .strafeLeft(25)
                .back(15)








                .build();
        RightClaw.setPosition(0.5);
        LeftClaw.setPosition(0.5);
        sleep(500);
        drive.followTrajectorySequence(traj1);
        RightClaw.setPosition(0.8);
        sleep(500);
        drive.followTrajectorySequence(traj2);
        sleep(200);
        ArmUpDown.setTargetPosition(1500);
        sleep(2000);
        LeftClaw.setPosition(0.3);
        sleep(1000);
        ArmUpDown.setTargetPosition(0);
        sleep(800);
        drive.followTrajectorySequence(traj3);

    }
}
