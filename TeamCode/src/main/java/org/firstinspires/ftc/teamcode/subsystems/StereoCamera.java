package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

public class StereoCamera extends Subsystem {
    double camera_distance= .17;
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
        HuskyLens.Block right_camera_april_tag = this.right_camera.findGoalAprilTag();
        HuskyLens.Block left_camera_april_tag = this.left_camera.findGoalAprilTag();
        if (right_camera_april_tag == null || left_camera_april_tag == null) return -1;
        telemetry.addData("left X", left_camera_april_tag.x);
        telemetry.addData("right X", right_camera_april_tag.x);

        double x_distance = 0;
        double numerator = this.camera_distance * this.right_camera.view_angle;
        double denominator = 2 * Math.tan(this.right_camera.view_angle/2) * (left_camera_april_tag.x - right_camera_april_tag.x);
        telemetry.addData("denominator", denominator);
        if (denominator == 0) {
            return x_distance;
        }
        x_distance = numerator / denominator;
        return x_distance/.0024;
    }

    public double findAngleToAprilTag() {
        HuskyLens.Block tag_left = this.left_camera.findGoalAprilTag();
        HuskyLens.Block tag_right = this.right_camera.findGoalAprilTag();
        int d_x  = 0;
        int id = 0;
        if (tag_right == null && tag_left == null) {return 0;}

        if (tag_right == null ) {
             d_x =  320 - tag_left.x;
             id = tag_left.id
;        }
        else if (tag_left == null) {
             d_x =  tag_right.x;
            id = tag_right.id;

        }
        else if ( tag_right.id > 3) {
             d_x = 320 - tag_left.x - tag_right.x;
            id = tag_right.id;

        }

        double difference = .0024*d_x;
        if (Math.abs(difference) <= .01 || id <= 3) {
            return 0;
        }
        return difference;
    }
}


