package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "Official_Main_Drive_CC", group = "")
//@Disabled
public class Official_Main_Drive_CC extends LinearOpMode {
    //private ElapsedTime runtime = new ElapsedTime ();
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private DcMotor RearLeft;
    private DcMotor ClimberRight;
    private DcMotor ClimberLeft;
    private DcMotor ArmInOut;
    private DcMotor ArmUpDown;


    private Servo LeftClaw;
    private Servo RightClaw;
    private Servo DroneLauncher;
  /*private DcMotor RightMotor;
  private DcMotor LeftMotor;*/

    private BNO055IMU imu;
    private boolean temp;
    private int count;
    boolean FC = true;
    double SpeedReducer = 0;


    //declare motor speed variables
    double RF, LF, RR, LR;

    //declare joystick position variables
    double X1, Y1, X2, Y2;

    //operational constants
    double joyScale = 0.7;  //0.5;

    double motorMax = 0.7;  //0.6;
    double Left_Stick_Angle, Left_Stick_Ratio, Left_Stick_Magnitude;
    double Left_Stick_Y, Left_Stick_X;
    double Robot_Angle, Output_Angle;
    double LTrigger = 0;
    int Count = 0;
    boolean LeftBumperIsPressed, RightBumperIsPressed, LeftClawClamped, RightClawClamped;

    @Override
    public void runOpMode () {

        FrontRight=hardwareMap.dcMotor.get("rightFront");
        FrontLeft=hardwareMap.dcMotor.get("leftFront");
        RearRight=hardwareMap.dcMotor.get("rightRear");
        RearLeft=hardwareMap.dcMotor.get("leftRear");

        ClimberLeft = hardwareMap.get(DcMotor.class, "ClimberLeft");
        ClimberRight = hardwareMap.get(DcMotor.class, "ClimberRight");
        ArmInOut = hardwareMap.get(DcMotor.class, "ArmInOut");
        ArmUpDown = hardwareMap.get(DcMotor.class, "ArmUpDown");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");


        ClimberLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClimberLeft.setTargetPosition(0);
        ClimberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ClimberRight.setDirection(DcMotor.Direction.REVERSE);
        ClimberRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClimberRight.setTargetPosition(0);
        ClimberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmInOut.setDirection(DcMotor.Direction.REVERSE);
        ArmInOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmInOut.setTargetPosition(0);
        ArmInOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);

        RightClaw.setDirection(Servo.Direction.REVERSE);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmInOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ClimberLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ClimberRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
  /*RightMotor = hardwareMap.dcMotor.get("RightMotor");
  LeftMotor = hardwareMap.dcMotor.get("LeftMotor");*/

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

        //RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if (opModeIsActive()) {

            ArmUpDown.setPower(0.5);
            ArmInOut.setPower(1);
            ClimberLeft.setPower(1);
            ClimberRight.setPower(1);
            while (opModeIsActive()) {

                if(gamepad1.right_stick_button)
                {
                    //FC = false;
                }
                if(gamepad1.left_stick_button)
                {
                    FC = true;
                }


                if(FC == true)
                {
                    LF = 0; RF = 0; LR = 0; RR = 0; X1 = 0; Y1 = 0;

                    //Setting up Variables
    /*if(gamepad1.a )
{
   motorMax = 0.3;  //0.5;
}
else
{*/
                    motorMax = 1;
//}
                    Left_Stick_Y = -gamepad1.left_stick_y;
                    Left_Stick_X = gamepad1.left_stick_x;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    Robot_Angle = angles.firstAngle * -1;
                    if(Left_Stick_Y != 0 || Left_Stick_X != 0)
                    {
                        Left_Stick_Ratio = Left_Stick_X / Left_Stick_Y;



                        //if left stick y greater than 0
                        if(Left_Stick_Y > 0){
      /*it creates this ratio left stick x/ left stick y, then it calulates the angle
      this is the same thing for the false just add 180 to the angle*/
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio));
                        }
                        else{
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio)) + 180;
                            if(Left_Stick_Angle > 180){Left_Stick_Angle -= 360;}
                        }
                        //it calculates the power in which direction based on the x and y
                        Left_Stick_Magnitude = Math.sqrt(Math.pow(Left_Stick_Y,2)
                                + Math.pow(Left_Stick_X,2));

                        //output angle is the way the robot wil go based on the joystick angle - the current robot angle
                        //the lines after it are just implementing them
                        Output_Angle = Left_Stick_Angle - Robot_Angle;
                        if(Output_Angle > 180){Output_Angle -= 360;}
                        if(Output_Angle < -180){Output_Angle += 360;}
                        //LTrigger = (1 - gamepad1.left_trigger);
                        //LTrigger = Math.max(LTrigger, 0.2);

                        //this will set a value for the x and y axis of the motor
                        Y1 = Math.cos(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;
                        X1 = Math.sin(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;

                    }
                    X2 = gamepad1.right_stick_x * joyScale;

                    // Forward/back movement
                    LF += Y1; RF += Y1; LR += Y1; RR += Y1;

                    //Side to side movement
                    LF += X1; RF -= X1; LR -= X1; RR += X1;

                    //Rotation Movement
                    LF += X2; RF -= X2; LR += X2; RR -= X2;

                    //Motor Speed

                    //Clip motor power values to +/- motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));


                    //Send values to the motors
                    /*if(gamepad1.left_trigger > gamepad2.left_trigger)
                    {
                        LTrigger = (0.75 - gamepad1.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }
                    else
                    {
                        LTrigger = (0.75 - gamepad2.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }*/

                    FrontLeft.setPower(LF * LTrigger);
                    FrontRight.setPower(RF * LTrigger);
                    RearLeft.setPower(LR * LTrigger);
                    RearRight.setPower(RR * LTrigger);


                }
                if(FC == false)
                {
                    LF = 0; RF = 0; LR = 0; RR = 0;
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();

                    X2 = gamepad1.right_stick_x * joyScale;


                    if(gamepad1.left_trigger > gamepad1.left_trigger)
                    {
                        LTrigger = (0.75 - gamepad1.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }
                    else
                    {
                        LTrigger = (0.75 - gamepad2.left_trigger);
                        LTrigger = Math.max(LTrigger, 0.2);
                    }
                    //Get joystick values
                    Y1 = -gamepad1.left_stick_y * joyScale; //invert so up is positive
                    X1 = gamepad1.left_stick_x * joyScale;
                    Y2 = -gamepad1.right_stick_y * joyScale;  //Y2 is not used at present


                    // Forward/back movement
                    LF += Y1; RF += Y1; LR += Y1; RR += Y1;

                    //Side to side movement
                    LF += X1; RF -= X1; LR -= X1; RR += X1;

                    //Rotation Movement
                    LF += X2; RF -= X2; LR += X2; RR -= X2;

                    motorMax = 1;

                    //Clip motor power values to +/- motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));


                    //Send values to the motors
                    FrontLeft.setPower(LF * LTrigger);
                    FrontRight.setPower(RF * LTrigger);
                    RearLeft.setPower(LR * LTrigger);
                    RearRight.setPower(RR * LTrigger);




                }
                if(gamepad1.dpad_up | gamepad2.dpad_up)
                {

                    ArmUpDown.setTargetPosition(1300);
                    LTrigger = 0.2;
                    if( ArmUpDown.getTargetPosition() > 300)
                    {

                        ArmInOut.setTargetPosition(1500);
                    }

                }
                else if(gamepad1.dpad_left | gamepad2.dpad_left)
                {

                    ArmUpDown.setTargetPosition(700);
                    ArmInOut.setTargetPosition(0);
                    LTrigger = 0.2;
                }
                else if(gamepad1.dpad_down | gamepad2.dpad_down)
                {

                    ArmInOut.setTargetPosition(0);

                    ArmUpDown.setTargetPosition(0);
                    LTrigger = 0.2;
                }
                else
                {

                    LTrigger = 1;
                    ArmInOut.setTargetPosition(0);
                    if( ArmInOut.getCurrentPosition() < 100)
                    {
                        ArmUpDown.setTargetPosition(100);
                    }
                }

                if(gamepad1.back | gamepad2.back)
                {
                    DroneLauncher.setPosition(0.5);
                }
                else
                {
                    DroneLauncher.setPosition(0);
                }

                if(gamepad1.left_bumper && LeftBumperIsPressed == false)
                {
                    if (LeftClawClamped)
                    {
                        LeftClaw.setPosition(gamepad1.left_trigger*0.3);
                        LeftClawClamped = false;
                        LeftBumperIsPressed = true;
                    } else
                    {
                        LeftClaw.setPosition(0.3);
                        LeftClawClamped = true;
                        LeftBumperIsPressed = true;
                    }
                }

                if (LeftClawClamped == false)
                {
                    LeftClaw.setPosition(gamepad1.left_trigger*0.3);
                }

                if (gamepad1.left_bumper == false)
                {
                    LeftBumperIsPressed = false;
                }

                if(gamepad1.right_bumper && RightBumperIsPressed == false)
                {
                    if (RightClawClamped)
                    {
                        RightClaw.setPosition(gamepad1.right_trigger*0.3);
                        RightClawClamped = false;
                        RightBumperIsPressed = true;
                    } else
                    {
                        RightClaw.setPosition(0.3);
                        RightClawClamped = true;
                        RightBumperIsPressed = true;
                    }
                }

                if (RightClawClamped == false)
                {
                    RightClaw.setPosition(gamepad1.right_trigger*0.3);
                }

                if (gamepad1.right_bumper == false)
                {
                    RightBumperIsPressed = false;
                }


                if(gamepad1.right_stick_button | gamepad2.right_stick_button)
                {
                    ClimberLeft.setTargetPosition(1100);
                    ClimberRight.setTargetPosition(2300);
                }
                else
                {
                    ClimberLeft.setTargetPosition(50);
                    ClimberRight.setTargetPosition(0);
                }

                if(gamepad1.left_stick_button | gamepad2.left_stick_button)
                {
                    ClimberLeft.setTargetPosition(-100);
                    ClimberRight.setTargetPosition(-100);
                }

                telemetry.addData("Arm In Out Target",ArmInOut.getTargetPosition());
                telemetry.addData("Arm In Out Current",ArmInOut.getCurrentPosition());
                telemetry.addLine("");
                telemetry.addData("Arm Up Down Target",ArmUpDown.getTargetPosition());
                telemetry.addData("Arm Up Down Current",ArmUpDown.getCurrentPosition());
                /*telemetry.addLine("");
                telemetry.addData("Left Bumper",gamepad1.left_bumper);
                telemetry.addData("Left Bumper ispressed",ispressed);
                telemetry.addData("Left Bumper Clamped",LeftClawClamped);*/
                telemetry.addLine("");
                telemetry.addData("Robot Angle",Robot_Angle);
                telemetry.update();


            }
        }
    }
}


