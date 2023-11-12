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

@TeleOp(name = "ClimberTest", group = "")
//@Disabled
public class ClimberTest extends LinearOpMode {

    private DcMotor ClimberRight;
    private DcMotor ClimberLeft;

    @Override
    public void runOpMode() {

        ClimberLeft = hardwareMap.get(DcMotor.class, "ClimberLeft");
        ClimberRight = hardwareMap.get(DcMotor.class, "ClimberRight");

        ClimberLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClimberLeft.setTargetPosition(0);
        ClimberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ClimberRight.setDirection(DcMotor.Direction.REVERSE);
        ClimberRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClimberRight.setTargetPosition(0);
        ClimberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ClimberLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ClimberRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
            /*ClimberLeft.setPower(1);
            ClimberRight.setPower(1);*/

                telemetry.addData("RightClimber", ClimberRight.getCurrentPosition());
                telemetry.addData("LeftClimber", ClimberLeft.getCurrentPosition());
                telemetry.update();

            }
        }
    }
}