package org.firstinspires.ftc.teamcode;

public class Constants {

    static double MAX_POWER = 1.0;
    static double MIN_POWER = -1.0;
    static double CHANGE_DIR = 1;
    static double WHEEL_DIST_H = 1.5;   //Horizontal distance from wheel to center 1.875
    static double WHEEL_DIST_V = 1.5;    // Vertical Distance from wheel to center
    static double COUNTS_PER_REV = 200;
    static double PLANET_GEAR_MECH = 1/16;
    static double WHEEL_DIAM = 4;
    static double COUNTS_PER_INCH =  (COUNTS_PER_REV*PLANET_GEAR_MECH) / (WHEEL_DIAM*Math.PI);
    static double XSCALE = 1;
    static double XY_RATIO = 1;

}

