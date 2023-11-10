package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Victo on 1/17/2018.
 */

public abstract class NavigationHardware {
    public NavigationHardware(HardwareMap map, String name){

    }
    public abstract double getHeading();

    public abstract double getError(double targetAngle);
}