package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmAndClawPosition {

    public DcMotor ArmUpDown;

    public DcMotor ArmInOut;

    public Servo LeftClaw;

    public static final double RightClawClosed = 0.3;
    public static final double LeftClawClosed = 0.3;

    public static final double RightClawOpen = 0;
    public static final double LeftClawOpen = 0;

    public static final int ArmUpDownHigh = 590;

    public static final int ArmInOutHigh = 1590;

    public static final int ArmUpDownMid = 550;

    public static final int ArmInOutMid = 1235;

    public static final int ArmUpDownLow = 444;

    public static final int ArmInOutLow = 773;

    public static final int ArmUpDownGround = 120;

    public static final int ArmInOutGround = 0;

    public static final int ArmUpDownRest = 200;

    public static final int ArmInOutRest = 0;

    public static final int ArmInOutOfficalAprilTag = 1500;


}