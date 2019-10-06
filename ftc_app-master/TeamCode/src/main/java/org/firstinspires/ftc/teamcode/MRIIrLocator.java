/*
DO NOT EDIT THIS PROGRAM

Modern Robotics IR Locator 360 Driver
Created 7/27/2017 by Colton Mehlhoff of Modern Robotics using FTC SDK 3.10
Reuse permitted with credit where credit is due

This class provides functions to use the IR Locator 360 http://modernroboticsinc.com/ir-locator-360
Support is available by emailing support@modernroboticsinc.com
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

public class MRIIrLocator {

    private byte[] chache;

    private I2cDevice sensor;
    private I2cDeviceSynch reader;

    HardwareMap hwMap = null;


    public MRIIrLocator() {

    }

    public void init(HardwareMap ahwMap, String cfgName) {
        init(ahwMap, cfgName, 0x1C); //Default I2C address for color beacon is 0x4c
    }

    public void init(HardwareMap ahwMap, String cfgName, int i2cAddr8) {

        hwMap = ahwMap;

        sensor = hwMap.i2cDevice.get(cfgName);

        reader = new I2cDeviceSynchImpl(sensor, I2cAddr.create8bit(i2cAddr8), false);

        reader.engage();
    }

    byte[] readAll(){
        chache = reader.read(0x00, 8);

        return chache;
    }

    int heading1200hz(){
        return (reader.read8(0x04)&0xFF) * 5;
    }

    int strength1200hz(){
        return reader.read8(0x05) & 0xFF;
    }

    int heading600hz(){
        return (reader.read8(0x06) & 0xFF) * 5;
    }

    int strength600hz(){
        return reader.read8(0x07) & 0xFF;
    }
}

