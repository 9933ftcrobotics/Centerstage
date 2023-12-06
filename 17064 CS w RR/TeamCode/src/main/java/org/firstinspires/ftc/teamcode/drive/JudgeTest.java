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

import org.firstinspires.ftc.teamcode.ArmAndClawPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "JudgeTest")
@Disabled
public class JudgeTest extends LinearOpMode {
    private DcMotor ArmUpDown;

    @Override
    public void runOpMode() throws InterruptedException {

        ArmUpDown=hardwareMap.dcMotor.get("ArmUpDown");

        ArmUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmUpDown.setPower(1);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest);

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        TrajectorySequence trajseq = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(0, 38, Math.toRadians(0)))
                .strafeRight(38)
                .turn(Math.toRadians(90))
                .build();


        drive.followTrajectorySequence(trajseq);

    }
}
