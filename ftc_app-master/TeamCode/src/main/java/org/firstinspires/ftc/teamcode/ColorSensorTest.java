package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name ="Color Test",group = "Auto")
//@Disabled
public class ColorSensorTest extends LinearOpMode {
    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;
    private ColorSensor colorSensor = null;
    public void runOpMode()
    {
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

        LFD = hardwareMap.get(DcMotor.class,"FL");
        RFD = hardwareMap.get(DcMotor.class,"FR");
        LBD = hardwareMap.get(DcMotor.class,"BL");
        RBD = hardwareMap.get(DcMotor.class,"BR");
        colorSensor.enableLed(true);
        telemetry.addData("Blueness",(colorSensor.blue()*2)/(double)((colorSensor.red()+colorSensor.green()+1)));

        waitForStart();
        while(opModeIsActive()&&(double)(colorSensor.blue()*2)/(double)((colorSensor.red()+colorSensor.green()+1))<0.5) {
            double[] dirs = HolonomicDrive.RoboMoveXY(-1, 0);
            LFD.setPower(dirs[0]);
            RFD.setPower(dirs[1]);
            LBD.setPower(dirs[2]);
            RBD.setPower(dirs[3]);
        }

        LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);




    }

}
