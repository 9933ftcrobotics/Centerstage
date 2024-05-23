package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ArmAndClawPosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ArmAndClawPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.HardWearMap;

import java.lang.Math;


@TeleOp(name = "ShooterRobotAprilTag", group = "")

public class ShooterRobotAprilTag extends LinearOpMode {
    //private ElapsedTime runtime = new ElapsedTime ();

    IMU imu;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private DcMotor RearLeft;

    private DcMotor belt;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;

    private Servo stopper;

    private TouchSensor touch;
 /*private DcMotor RightMotor;
 private DcMotor LeftMotor;*/


    //private BNO055IMU imu;
    private boolean temp;
    private int count;
    boolean FC = true;
    double SpeedReducer = 0;

    int MotorSpeed = 3300;
    int start = 0;
    boolean YisPressed = false;
    boolean AisPressed = false;



    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    final double DESIRED_DISTANCE = 60; //12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.5  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;




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
    double RightClawPositionClosed = 0.21;
    double LeftClawPositionClosed = 0.27;
    double Speed = 1;
    int Count = 0;
    boolean LeftBumperIsPressed, RightBumperIsPressed, LeftClawClamped, RightClawClamped, Climbed;


    boolean OverRide = false;


    double normal = 0.7;

    double LTrigger = 0.7;


    double slow = 0.3;

    public int ArmUpDownMidLocal = 550;

    boolean XisPressed = false;

    boolean BisPressed = false;


    @Override
    public void runOpMode () {

        //imu.resetYaw();

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        //double  driveTwo           = 0;        // Desired forward power/speed (-1 to +1)
        double drive = 0;
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        motorOne.setDirection(DcMotor.Direction.REVERSE);
        belt.setDirection(DcMotor.Direction.REVERSE);


        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);


        //LeftClaw.setDirection(Servo.Direction.REVERSE);


        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;
        imu = hardwareMap.get(IMU.class, "imu");
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

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));


        //RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                targetFound = false;
                desiredTag = null;

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }


                if (gamepad1.right_stick_button) {
                    //FC = false;
                }
                if (gamepad1.left_stick_button) {
                    FC = true;
                }





                if (!gamepad1.x) {
                    if (gamepad1.y && !YisPressed) {
                        MotorSpeed = MotorSpeed + 100;
                        YisPressed = true;
                    } else if (gamepad1.a && !AisPressed) {
                        MotorSpeed = MotorSpeed - 100;
                        AisPressed = true;
                    }

                    if (MotorSpeed > 6000) {
                        MotorSpeed = 6000;
                    } else if (MotorSpeed < 100) {
                        MotorSpeed = 100;
                    }

                    if (gamepad1.right_bumper && !RightBumperIsPressed) {
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

                    if (start == 1) {
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
                }


                if (targetFound == true) {

                }


                if (targetFound) {
                    telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
                }


                //If a is pressed (meaning driver wants to score) and robot can see an april tag, then score
                if (gamepad1.x && targetFound) {


                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double rangeError = (-desiredTag.ftcPose.range + DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                    moveRobot(drive, strafe, turn);

                    if (rangeError < 3 && rangeError > -3) {
                        motorOne.setVelocity(MotorSpeed);
                        motorTwo.setVelocity(MotorSpeed * 0.5);
                        sleep(500);
                        stopper.setPosition(0.5);
                        if (stopper.getPosition() > 0.4) {
                            belt.setPower(1);
                        }
                    }
                } else {
                    telemetry.addData("in driver control", FC);
                    telemetry.addData("Right Front Power", FrontRight.getPower());
                    LF = 0;
                    RF = 0;
                    LR = 0;
                    RR = 0;
                    X1 = 0;
                    Y1 = 0;


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
                    imu.getRobotYawPitchRollAngles();
                    imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                    Robot_Angle = orientation.getYaw(AngleUnit.DEGREES) * -1;
                    if (Left_Stick_Y != 0 || Left_Stick_X != 0) {
                        Left_Stick_Ratio = Left_Stick_X / Left_Stick_Y;


                        //if left stick y greater than 0
                        if (Left_Stick_Y > 0) {
     /*it creates this ratio left stick x/ left stick y, then it calulates the angle
     this is the same thing for the false just add 180 to the angle*/
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio));
                        } else {
                            Left_Stick_Angle = Math.toDegrees(Math.atan(Left_Stick_Ratio)) + 180;
                            if (Left_Stick_Angle > 180) {
                                Left_Stick_Angle -= 360;
                            }
                        }
                        //it calculates the power in which direction based on the x and y
                        Left_Stick_Magnitude = Math.sqrt(Math.pow(Left_Stick_Y, 2)
                                + Math.pow(Left_Stick_X, 2));


                        //output angle is the way the robot wil go based on the joystick angle - the current robot angle
                        //the lines after it are just implementing them
                        Output_Angle = Left_Stick_Angle - Robot_Angle;
                        if (Output_Angle > 180) {
                            Output_Angle -= 360;
                        }
                        if (Output_Angle < -180) {
                            Output_Angle += 360;
                        }
                        //Speed = (1 - gamepad1.left_trigger);
                        //Speed = Math.max(Speed, 0.2);


                        //this will set a value for the x and y axis of the motor
                        Y1 = Math.cos(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;
                        X1 = Math.sin(Math.toRadians(Output_Angle)) * Left_Stick_Magnitude;


                    }
                    X2 = gamepad1.right_stick_x * joyScale;


                    // Forward/back movement
                    LF += Y1;
                    RF += Y1;
                    LR += Y1;
                    RR += Y1;


                    //Side to side movement
                    LF += X1;
                    RF -= X1;
                    LR -= X1;
                    RR += X1;


                    //Rotation Movement
                    LF += X2;
                    RF -= X2;
                    LR += X2;
                    RR -= X2;


                    //Motor Speed


                    //Clip motor power values to +/- motorMax
                    LF = Math.max(-motorMax, Math.min(LF, motorMax));
                    RF = Math.max(-motorMax, Math.min(RF, motorMax));
                    LR = Math.max(-motorMax, Math.min(LR, motorMax));
                    RR = Math.max(-motorMax, Math.min(RR, motorMax));


                    //Send values to the motors
                   /*if(gamepad1.left_trigger > gamepad2.left_trigger)
                   {
                       Speed = (0.75 - gamepad1.left_trigger);
                       Speed = Math.max(Speed, 0.2);
                   }
                   else
                   {
                       Speed = (0.75 - gamepad2.left_trigger);
                       Speed = Math.max(Speed, 0.2);
                   }*/


                    FrontLeft.setPower(LF * Speed);
                    FrontRight.setPower(RF * Speed);
                    RearLeft.setPower(LR * Speed);
                    RearRight.setPower(RR * Speed);
                }

                // Step through the list of detected tags and look for a matching tag


                //double c = Math.asin(b);
                //System.out.println(Math.asin(c));


                telemetry.addData("Power", normal);
                telemetry.addData("Slow", slow);
                /*if (Speed == slow) {
                    telemetry.addLine("Speed is Slow");
                }*/
               /*telemetry.addLine("");
               telemetry.addData("LeftClaw",LeftClaw.getPosition());


               telemetry.addLine("");
               telemetry.addData("Left Bumper",gamepad1.left_bumper);
               telemetry.addData("Left Bumper ispressed",ispressed);
               telemetry.addData("Left Bumper Clamped",LeftClawClamped);*/
                telemetry.addLine("");
                telemetry.addData("Robot Angle", Robot_Angle);
                telemetry.update();


            }

        }
    }



    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        FrontLeft.setPower(leftFrontPower);
        FrontRight.setPower(rightFrontPower);
        RearLeft.setPower(leftBackPower);
        RearRight.setPower(rightBackPower);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}












