/*
Modern Robotics IR Locator 360 Example
Created 7/27/2017 by Colton Mehlhoff of Modern Robotics using FTC SDK 3.10
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "irl" (MRI IR Locator 360 with default I2C address 0x1C)

MRIIrLocator class must be in the same folder as this program. Download from http://modernroboticsinc.com/ir-locator-360

To change I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IR Locator Ex", group = "MRI")
@Disabled
public class MRI_Ir_Locator extends LinearOpMode {

    MRIIrLocator locator = new MRIIrLocator();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        locator.init(hardwareMap, "irl");  //initializes the I2CDevice. Second parameter is the name of the sensor in the configuration file.

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Dir 1200", locator.heading1200hz());
            telemetry.addData("Dir 600", locator.heading600hz());
            telemetry.addData("Str 1200", locator.strength1200hz());
            telemetry.addData("Str 600", locator.strength600hz());
            telemetry.update();
        }
    }
}
