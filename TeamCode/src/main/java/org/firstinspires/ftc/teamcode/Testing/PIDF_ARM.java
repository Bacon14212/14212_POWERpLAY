package org.firstinspires.ftc.teamcode.Testing;

import com.arcrobotics.ftclib.controller.PIDController;

public class PIDF_ARM {
    private PIDController controller;

    public static double  p = 0, i = 0, d = 0;
    public static double  f = 0;

    public static int target = 0;

    private static final double ticks_in_degree = 700 / 180.0;


}
