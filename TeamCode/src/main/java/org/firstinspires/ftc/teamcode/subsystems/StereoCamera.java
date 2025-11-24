package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

public class StereoCamera extends Subsystem {
    double camera_distance;
    Camera left_camera;
    Camera right_camera;
    public StereoCamera( Camera left_camera, Camera right_camera, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.left_camera = left_camera;
        this.right_camera = right_camera;
    }
    public static class StereoConfig {
        public static double x_distance = 1;
        public static double y_distance = 1;
        public static double z_distance = 1;

    }
    public double compute_x_distance( )
    {
        HuskyLens.Block right_camera_april_tag = this.right_camera.find_april_tag();
        HuskyLens.Block left_camera_april_tag = this.left_camera.find_april_tag();
        if (right_camera_april_tag == null || left_camera_april_tag == null) return -1;

        double x_distance = 0;
        double numerator = this.camera_distance * this.right_camera.view_angle;
        double denominator = 2 * Math.tan(this.right_camera.view_angle/2) * (left_camera_april_tag.x - right_camera_april_tag.x);
        if (denominator == 0) {
            return x_distance;
        }
        x_distance = numerator / denominator;
        return x_distance;
    }
}


