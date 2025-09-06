package org.firstinspires.ftc.teamcode.subsystems;

public class stereo_camera {
    double camera_distance;
    camera left_camera;
    camera right_camera;
    public static class StereoConfig {
        public static double x_distance = 1;
        public static double y_distance = 1;
        public static double z_distance = 1;

    }

    double compute_x_distance( )
    {
        april_tag_location right_camera_april_tag = this.right_camera.find_april_tag();
        april_tag_location left_camera_april_tag = this.left_camera.find_april_tag();
        double x_distance = 0;
        double numerator = this.camera_distance * this.right_camera.view_angle;
        double denominator = 2 * Math.tan(this.right_camera.view_angle/2) * (left_camera_april_tag.x - right_camera_april_tag.x);
        return x_distance;
    }
}


