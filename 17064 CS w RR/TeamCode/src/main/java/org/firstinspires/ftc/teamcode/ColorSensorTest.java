package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "ColorSensorTest (Blocks to Java)")
@Disabled
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("Color Sensor Red", colorSensor.red());
                telemetry.addData("Color Sensor Green", colorSensor.green());
                telemetry.addData("Color Sensor Blue", colorSensor.blue());
                telemetry.update();
            }
        }
    }
}