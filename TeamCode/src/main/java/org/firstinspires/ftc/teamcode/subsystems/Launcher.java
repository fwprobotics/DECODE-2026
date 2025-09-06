package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.NaN;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.Vector;

public class Launcher {
    public double distance_to_velocity (float x_pos, float theta, float target_pos, float tolerance){
        double Mag_V = 0;
        double numerator = 7 * x_pos * (1/Math.cos(theta));
        double denominator = Math.sqrt(10) * Math.sqrt(x_pos*Math.tan(theta)-target_pos-tolerance);
        if (denominator == 0) {
            return  NaN;
        }
        return Mag_V;
    };
}
