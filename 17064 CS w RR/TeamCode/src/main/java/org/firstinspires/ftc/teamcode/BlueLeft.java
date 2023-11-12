package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "BlueLeft", group = "")
//@Disabled
public class BlueLeft extends LinearOpMode {

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor RearRight;
    private DcMotor RearLeft;
    private Servo RightClaw;
    private DcMotor ArmUpDown;
    private DistanceSensor LeftDistance;
    private DistanceSensor RightDistance;
    double Next = 0;
    int Ticks = 0;
    int Ticks2 = 0;
    int Distance2;


    @Override
    public void runOpMode() {


        //Put Motor Name = dcMotor Motor Name here
        FrontRight=hardwareMap.dcMotor.get("rightFront");
        FrontLeft=hardwareMap.dcMotor.get("leftFront");
        RearRight=hardwareMap.dcMotor.get("rightRear");
        RearLeft=hardwareMap.dcMotor.get("leftRear");
        RightClaw=hardwareMap.servo.get("RightClaw");
        ArmUpDown=hardwareMap.dcMotor.get("ArmUpDown");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        int Step = 0;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);




        waitForStart();
        if (opModeIsActive()) {
            //Intilazation code here
            FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            ArmUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmUpDown.setTargetPosition(0);
            ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (opModeIsActive()) {
                ArmUpDown.setPower(0.5);
                ArmUpDown.setTargetPosition(50);
                angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


                FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                /*FrontLeft.setPower(1);
                FrontRight.setPower(1);
                RearLeft.setPower(1);
                RearRight.setPower(1);*/



                telemetry.addData("Left", LeftDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Right", RightDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
                int Marker = 1;


                if(LeftDistance.getDistance(DistanceUnit.CM) < 100)
                {
                    Marker = 2;
                }
                else if(RightDistance.getDistance(DistanceUnit.CM) < 100)
                {
                    Marker = 0;
                }
                sleep(1000);

                /*RightClaw.setPosition(0.2); //middle spike
                sleep(2000);
                Drive(-4500,0.5);*/

                /*sleep(2000); //right spike
                Drive(-3800,0.5);
                Turn(20,0,0.5);
                Drive(-500,0.5);
                 */
                /*sleep(2000);
                Drive(-8600,0.5);
                Turn(-90,0,0.3);
                Drive(-100,0.5);
                sleep(500);
                Drive(200,0.5);
                Turn(0,0,0.3);*/


                if(Marker==0)
                {
                    //Left
                    /*Drive(-2500,0.5);
                    Turn(-30,-60,0.5);
                    Drive(-1700,0.5);
                    RightClaw.setPosition(1);
                    sleep(2000);
                    Drive(1700,0.5);
                    Turn(-30,0,0.5);
                    Drive(2500,0.5);*/
                    RightClaw.setPosition(0.3);
                    sleep(500);
                    Drive(-3800,0.5);
                    Turn(-20,0,0.3);
                    Drive(-200,0.5);
                    sleep(500);
                    RightClaw.setPosition(0);
                    Drive(200,0.5);
                    Turn(0,0,0.3);
                }
                else if(Marker==2)
                {
                    //Right
                    /*Turn(0,20,0.5);
                    Drive(-1250,0.5);
                    RightClaw.setPosition(1);
                    sleep(2000);
                    Drive(1250,0.5);
                    Turn(20,0,0.5);*/
                    RightClaw.setPosition(0.3);
                    sleep(500); //right spike
                    Drive(-8600,0.5);
                    Turn(90,0,0.5);
                    Drive(-500,0.5);
                    RightClaw.setPosition(0);
                    Drive(500,0.5);
                    Turn(0,0,0.5);


                }
                else
                {
                    //Center
                    /*Drive(-4000,0.5);
                    RightClaw.setPosition(1);
                    sleep(2000);
                    Drive(4000,0.5);*/
                    RightClaw.setPosition(0.3);
                    sleep(2000);
                    Drive(-4500,0.5);
                    RightClaw.setPosition(0);
                    Drive(500,0.5);
                }

                /*Drive(1700,0.5);
                Turn(-55,-90,0.5);
                Drive(-30000,0.7);*/








                angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Angle:",-angles.firstAngle);
                telemetry.update();


            }
        }
    }




    private void Turn(int Angle,int Angle2, double Power)
    {
        //TURN RIGHT IS POSITIVE
        //TURN LEFT IS NEGATIVE
        //initializeGyro();
        // Next = 0;

        telemetry.addLine("We are now inside Turn function");
        telemetry.update();

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float RobotAngle = -angles.firstAngle;
        telemetry.addData("Gyro angle", RobotAngle);
        telemetry.update();
        sleep(2000);
        //double Power=0;
        double Power2 = 0;
        Power2 = Power * 0.3;

        if (Angle < RobotAngle) {
            while (Angle < RobotAngle)
            {
                telemetry.addLine("Can you see me?");
                telemetry.addData("Mode",FrontRight.getMode() );
                telemetry.addData("Gyro",RobotAngle );
                telemetry.update();
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                RobotAngle = -angles.firstAngle;
                FrontLeft.setPower(-Power);
                FrontRight.setPower(Power);
                RearRight.setPower(Power);
                RearLeft.setPower(-Power);
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearRight.setPower(0);
            RearLeft.setPower(0);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //double Power=0;
            RobotAngle = -angles.firstAngle;
            Power2 = Power * 0.3;

            while (Angle2 < RobotAngle)
            {
                telemetry.addLine("Can you see me?");
                telemetry.addData("Mode",FrontRight.getMode() );
                telemetry.addData("Gyro",RobotAngle );
                telemetry.update();
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                RobotAngle = -angles.firstAngle;
                FrontLeft.setPower(-Power2);
                FrontRight.setPower(Power2);
                RearRight.setPower(Power2);
                RearLeft.setPower(-Power2);
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearRight.setPower(0);
            RearLeft.setPower(0);
            //Next = 1;
        }
        else {

            while (Angle > RobotAngle)
            {
                telemetry.addLine("Can you see me?");
                telemetry.addData("Mode",FrontRight.getMode() );
                telemetry.addData("Gyro",RobotAngle );
                telemetry.update();
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                RobotAngle = -angles.firstAngle;
                FrontLeft.setPower(Power);
                FrontRight.setPower(-Power);
                RearRight.setPower(-Power);
                RearLeft.setPower(Power);
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearRight.setPower(0);
            RearLeft.setPower(0);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //double Power=0;

            Power2 = Power * 0.3;

            while (Angle2 > RobotAngle)
            {
                telemetry.addLine("Can you see me?");
                telemetry.addData("Mode",FrontRight.getMode() );
                telemetry.addData("Gyro",RobotAngle );
                telemetry.update();
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                RobotAngle = -angles.firstAngle;
                FrontLeft.setPower(Power2);
                FrontRight.setPower(-Power2);
                RearRight.setPower(-Power2);
                RearLeft.setPower(Power2);
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearRight.setPower(0);
            RearLeft.setPower(0);
            //Next = 1;
        }


    }


    private void Drive(double Distance, double Power) {
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Next = 0;
        //Distance2 = Distance * 50;
        //     Ticks = Distance2 - 50;
        int NewDis = (int)(FrontLeft.getCurrentPosition() + Distance);
        telemetry.addData("New Dis",NewDis);
        telemetry.addData("Dis",Distance);
        telemetry.addData("Pos",FrontLeft.getCurrentPosition());
        telemetry.update();
        if(NewDis > FrontLeft.getCurrentPosition())
        {
            while(NewDis >= FrontLeft.getCurrentPosition())
            {
                telemetry.addData("New Dis",NewDis);
                telemetry.addData("Dis",Distance);
                telemetry.addData("Pos",FrontLeft.getCurrentPosition());
                telemetry.update();
                FrontLeft.setPower(-Power);
                FrontRight.setPower(-Power);
                RearLeft.setPower(-Power);
                RearRight.setPower(-Power);
            }
        }
        else
        {
            while(NewDis <= FrontLeft.getCurrentPosition())
            {
                telemetry.addData("New Dis",NewDis);
                telemetry.addData("Dis",Distance);
                telemetry.addData("Pos",FrontLeft.getCurrentPosition());
                telemetry.update();
                FrontLeft.setPower(Power);
                FrontRight.setPower(Power);
                RearLeft.setPower(Power);
                RearRight.setPower(Power);
            }
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
        //sleep(100);



        Next = 1;
    }



}



