package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

import java.lang.Math;

@TeleOp(name = "Shooter Robot", group = "")
// @Disabled
public class ShooterRobot extends LinearOpMode {
    // private ElapsedTime runtime = new ElapsedTime ();

    private ColorSensor color;
    private DcMotor belt;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;

    private Servo stopper;

    private TouchSensor touch;

    int MotorSpeed = 2900;
    int start = 0;
    boolean YisPressed = false;
    boolean AisPressed = false;

    //boolean RightBumper = false;

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private DcMotor RearLeft;

    private BNO055IMU imu;
    private boolean temp;
    private int count;
    boolean FC = true;
    double SpeedReducer = 0;

    // declare motor speed variables
    double RF, LF, RR, LR;

    // declare joystick position variables
    double X1, Y1, X2, Y2;

    // operational constants
    double joyScale = 0.7; // 0.5;

    double motorMax = 0.7; // 0.6;
    double Left_Stick_Angle, Left_Stick_Ratio, Left_Stick_Magnitude;
    double Left_Stick_Y, Left_Stick_X;
    double Robot_Angle, Output_Angle;
    double LTrigger = 0.7;
    int Count = 0;
    boolean LeftBumperIsPressed, RightBumperIsPressed, LeftClawClamped, RightClawClamped;

    @Override
    public void runOpMode() {

        FrontRight = hardwareMap.dcMotor.get("rightFront");
        FrontLeft = hardwareMap.dcMotor.get("leftFront");
        RearRight = hardwareMap.dcMotor.get("rightRear");
        RearLeft = hardwareMap.dcMotor.get("leftRear");

        stopper = hardwareMap.get(Servo.class, "stopper");

        belt = hardwareMap.get(DcMotor.class, "belt");
        motorOne = hardwareMap.get(DcMotorEx.class, "leftmotor");
        motorTwo = hardwareMap.get(DcMotorEx.class, "rightmotor");
        color = hardwareMap.get(ColorSensor.class, "color");

        touch = hardwareMap.get(TouchSensor.class, "touch");

        motorOne.setDirection(DcMotor.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.REVERSE);


        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);


        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setTargetPosition(0);
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        belt.setPower(1);

        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        /*
         * RightMotor = hardwareMap.dcMotor.get("RightMotor");
         * LeftMotor = hardwareMap.dcMotor.get("LeftMotor");
         */

        temp = true;
        count = 0;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

        // RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//motorOne.setVelocity(200);

//motorTwo.setTargetPosition(300);
// Switch to RUN_TO_POSITION mode
//motorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive()) {

            stopper.setPosition(0.5);

            while (opModeIsActive()) {

                if (gamepad1.right_stick_button) {
                    // FC = false;
                }
                if (gamepad1.left_stick_button) {
                    FC = true;
                }

                if (FC == true) {
                    LF = 0;
                    RF = 0;
                    LR = 0;
                    RR = 0;
                    X1 = 0;
                    Y1 = 0;

                    // Setting up Variables
                    /*
                     * if(gamepad1.a )
                     * {
                     * motorMax = 0.3; //0.5;
                     * }
                     * else
                     * {
                     */
                    motorMax = 1;
                    // }
                    Left_Stick_Y = -gamepad1.left_stick_y;
                    Left_Stick_X = gamepad1.left_stick_x;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    Robot_Angle = angles.firstAngle * -1;
                    if (Left_Stick_Y != 0 || Left_Stick_X != 0) {
                        Left_Stick_Ratio = Left_Stick_X / Left_Stick_Y;

                        // if left stick y greater than 0
                        if (Left_Stick_Y > 0) {
                            /*
                             * it creates this ratio left stick x/ left stick y, then it calulates the angle
                             * this is the same thing for the false just add 180 to the angle
                             */
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio));
                        } else {
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio)) + 180;
                            if (Left_Stick_Angle > 180) {
                                Left_Stick_Angle -= 360;
                            }
                        }
                        // it calculates the power in which direction based on the x and y
                        Left_Stick_Magnitude = Math.sqrt(Math.pow(Left_Stick_Y, 2)
                                + Math.pow(Left_Stick_X, 2));

                        // output angle is the way the robot wil go based on the joystick angle - the
                        // current robot angle
                        // the lines after it are just implementing them
                        Output_Angle = Left_Stick_Angle - Robot_Angle;
                        if (Output_Angle > 180) {
                            Output_Angle -= 360;
                        }
                        if (Output_Angle < -180) {
                            Output_Angle += 360;
                        }
                        // LTrigger = (1 - gamepad1.left_trigger);
                        // LTrigger = Math.max(LTrigger, 0.2);

                        // this will set a value for the x and y axis of the motor
                        Y1 = Math.cos(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;
                        X1 = Math.sin(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;

                    }
                    X2 = gamepad1.right_stick_x * joyScale;

                    // Forward/back movement
                    LF += Y1;
                    RF += Y1;
                    LR += Y1;
                    RR += Y1;

                    // Side to side movement
                    LF += X1;
                    RF -= X1;
                    LR -= X1;
                    RR += X1;

                    // Rotation Movement
                    LF += X2;
                    RF -= X2;
                    LR += X2;
                    RR -= X2;

                    // Motor Speed

                    // Clip motor power values to +/- motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));

                    // Send values to the motors
                    /*
                     * if(gamepad1.left_trigger > gamepad2.left_trigger)
                     * {
                     * LTrigger = (0.75 - gamepad1.left_trigger);
                     * LTrigger = Math.max(LTrigger, 0.2);
                     * }
                     * else
                     * {
                     * LTrigger = (0.75 - gamepad2.left_trigger);
                     * LTrigger = Math.max(LTrigger, 0.2);
                     * }
                     */

                    FrontLeft.setPower(LF * LTrigger);
                    FrontRight.setPower(RF * LTrigger);
                    RearLeft.setPower(LR * LTrigger);
                    RearRight.setPower(RR * LTrigger);

                }
                if (FC == false) {
                    LF = 0;
                    RF = 0;
                    LR = 0;
                    RR = 0;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();

                    X2 = gamepad1.right_stick_x * joyScale;

                    if (gamepad1.left_trigger > gamepad1.left_trigger) {
                        LTrigger = (0.75 - gamepad1.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    } else {
                        LTrigger = (0.75 - gamepad2.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }
                    // Get joystick values
                    Y1 = -gamepad1.left_stick_y * joyScale; // invert so up is positive
                    X1 = gamepad1.left_stick_x * joyScale;
                    Y2 = -gamepad1.right_stick_y * joyScale; // Y2 is not used at present

                    // Forward/back movement
                    LF += Y1;
                    RF += Y1;
                    LR += Y1;
                    RR += Y1;

                    // Side to side movement
                    LF += X1;
                    RF -= X1;
                    LR -= X1;
                    RR += X1;

                    // Rotation Movement
                    LF += X2;
                    RF -= X2;
                    LR += X2;
                    RR -= X2;

                    motorMax = 1;

                    // Clip motor power values to +/- motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));

                    // Send values to the motors
                    FrontLeft.setPower(LF * LTrigger);
                    FrontRight.setPower(RF * LTrigger);
                    RearLeft.setPower(LR * LTrigger);
                    RearRight.setPower(RR * LTrigger);

                }

                /*
                 * telemetry.addLine("");
                 * telemetry.addData("Left Bumper",gamepad1.left_bumper);
                 * telemetry.addData("Left Bumper ispressed",ispressed);
                 * telemetry.addData("Left Bumper Clamped",LeftClawClamped);
                 */

                if (gamepad1.y && !YisPressed) {
                    MotorSpeed = MotorSpeed + 100;
                    YisPressed = true;
                } else if (gamepad1.a && !AisPressed) {
                    MotorSpeed = MotorSpeed - 100;
                    AisPressed = true;
                }

                if (!gamepad1.a) {
                    AisPressed = false;
                }
                if (!gamepad1.y) {
                    YisPressed = false;
                }

                if (MotorSpeed > 6000) {
                    MotorSpeed = 6000;
                } else if (MotorSpeed < 100) {
                    MotorSpeed = 100;
                }

                if(gamepad1.right_bumper && !RightBumperIsPressed){
                    if (start == 0) {
                        start = 1;
                        RightBumperIsPressed = true;
                    } else if (start == 1) {
                        start = 0;
                        RightBumperIsPressed = true;
                    }
                }

                if (!gamepad1.right_bumper) {
                    RightBumperIsPressed = false;
                }

                //belt.setPower(gamepad1.right_stick_y);

                /*if (touch.isPressed()) {
                    belt.setPower(1);
                    //stopper.setPosition(0.5);
                } else if (!touch.isPressed()) {
                    belt.setPower(0);
                }*/

                if (gamepad1.right_trigger > 0.1 || touch.isPressed()) {
                    belt.setDirection(DcMotor.Direction.REVERSE);
                    belt.setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0.1) {
                    belt.setDirection(DcMotor.Direction.FORWARD);
                    belt.setPower(gamepad1.left_trigger);
                } else if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 || !touch.isPressed()) {
                    belt.setPower(0);
                }

                if(start == 1){
                    motorTwo.setVelocity(MotorSpeed * 0.25);
                    //motorTwo.setVelocity(MotorSpeed);
                    motorOne.setVelocity(MotorSpeed);
                    if (motorOne.getVelocity() > 1000) {
                        stopper.setPosition(0.5);
                    }
                } else if (start == 0) {
                    stopper.setPosition(0);
                    motorTwo.setVelocity(0);
                    motorOne.setVelocity(0);
                }

                telemetry.addData("Right Bumper", RightBumperIsPressed);

                telemetry.addData("Color Sensor red", color.red());
                telemetry.addData("Color Sensor green", color.green());
                telemetry.addData("Color Sensor blue", color.blue());
                telemetry.addData("RPM On Shooter:", MotorSpeed);
                telemetry.addData("Shooter On/Off. 0 = off 1 = on ", start);
                telemetry.addData("Y is Pressed", YisPressed);
                telemetry.addData("A is Pressed", AisPressed);
                telemetry.addLine("");
                telemetry.addData("Robot Angle", Robot_Angle);
                telemetry.addData("Joystick LY", -Left_Stick_Y);
                telemetry.update();

            }
        }
    }
}