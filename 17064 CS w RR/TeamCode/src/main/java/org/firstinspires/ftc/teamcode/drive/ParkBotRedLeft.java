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


@Autonomous(group = "ParkBotRedLeft")
@Disabled
public class ParkBotRedLeft extends LinearOpMode {
    private Servo RightClaw;
    private Servo LeftClaw;
    private DcMotor ArmUpDown;
    private DistanceSensor LeftDistance;
    private DistanceSensor RightDistance;

    @Override
    public void runOpMode() throws InterruptedException {

        RightClaw=hardwareMap.servo.get("RightClaw");
        LeftClaw=hardwareMap.servo.get("LeftClaw");
        ArmUpDown=hardwareMap.dcMotor.get("ArmUpDown");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        ArmUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmUpDown.setPower(1);

        LeftClaw.setDirection(Servo.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        RightClaw.setPosition(0.21);
        LeftClaw.setPosition(0.3);
        sleep(1000);
        ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest);

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
        TrajectorySequence trajseq = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                .forward(54)
                .strafeRight(95)
                .build();


        drive.followTrajectorySequence(trajseq);

    }
}
