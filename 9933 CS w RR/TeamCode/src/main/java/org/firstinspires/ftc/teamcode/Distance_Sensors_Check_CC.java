package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@TeleOp(name = "Distance_Sensors_Check_CC", group = "")
//@Disabled
@Config
public class Distance_Sensors_Check_CC extends LinearOpMode {
    String Spike;


    private DistanceSensor LeftDistance;
    private DistanceSensor RightDistance;

    @Override
    public void runOpMode () {
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {

                if(LeftDistance.getDistance(DistanceUnit.CM) < 110)
                {
                    Spike = "Right Spike";
                }
                else if(RightDistance.getDistance(DistanceUnit.CM) < 120)
                {
                    Spike = "Left Spike";
                }
                else
                {
                    Spike = "Center Spike";
                }




                //telemetry.addLine("");
                telemetry.addData("Left Distance", LeftDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Right Distance", RightDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Spike Selected", Spike);
                telemetry.update();


            }
        }
    }
}


