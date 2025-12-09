package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.ApriltagDetection;
import org.firstinspires.ftc.teamcode.pipelines.ApriltagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

public class Camera extends Subsystem {
    double view_angle;
    VisionPortal visionPortal;
    public HuskyLens webcam;

    public Camera(String Cameramap, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.view_angle = 2*Math.PI/3;
        webcam = hardwareMap.get(HuskyLens.class, Cameramap);
        webcam.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        initCV(Cameramap);
    }
    public void initCV(String cameramap) {}
    public HuskyLens.Block find_april_tag() {
        HuskyLens.Block[] blocks = webcam.blocks();
        if (blocks.length == 0) {
            telemetry.addData("FAILED TO FIND", blocks.length);
            return null;
        }
        HuskyLens.Block returnTag = blocks[0];
        for (HuskyLens.Block block : blocks) {
            if (block.width > returnTag.width) {
                returnTag = block;
            }
        }
        return returnTag;
    }
    public double getAreaOfAprilTag() {
        HuskyLens.Block aprilTag = find_april_tag();
        if (aprilTag == null) {
            return 0;
        }
        return aprilTag.width * aprilTag.height;
    }
    public static double GetDistanceFromArea(double Area) {
        return Math.sqrt(2700.43138/(Area - 14.09491));
    }
}




