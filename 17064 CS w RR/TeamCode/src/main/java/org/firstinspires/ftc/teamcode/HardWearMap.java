package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.apache.commons.math3.analysis.function.Asin;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class HardWearMap {

    public  DcMotor FrontLeft;
    public static DcMotor FrontRight;
    public static DcMotor RearRight;
    public static DcMotor RearLeft;

    public static DcMotor belt;
    public static DcMotorEx motorOne;
    public static DcMotorEx motorTwo;

    public static Servo stopper;

    public static TouchSensor touch;


        /*FrontRight = hardwareMap.dcMotor.get("rightFront");
        FrontLeft = hardwareMap.dcMotor.get("leftFront");
        RearRight = hardwareMap.dcMotor.get("rightRear");
        RearLeft = hardwareMap.dcMotor.get("leftRear");


        stopper = hardwareMap.get(Servo.class, "stopper");

        belt = hardwareMap.get(DcMotor.class, "belt");
        motorOne = hardwareMap.get(DcMotorEx.class, "leftmotor");
        motorTwo = hardwareMap.get(DcMotorEx.class, "rightmotor");
        //color = hardwareMap.get(ColorSensor.class, "color");

        touch = hardwareMap.get(TouchSensor.class, "touch");*/


}