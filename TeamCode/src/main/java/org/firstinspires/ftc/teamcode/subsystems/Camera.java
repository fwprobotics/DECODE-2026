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
        webcam = hardwareMap.get(HuskyLens.class, Cameramap);
        webcam.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        initCV(Cameramap);
    }


    public void initCV(String cameramap) {

    }
    public HuskyLens.Block find_april_tag() {
        HuskyLens.Block[] blocks = webcam.blocks();
        telemetry.addData("BLOCKS:", Arrays.toString(blocks));
        if (blocks.length == 0) {
            return null;
        }
        HuskyLens.Block returnTag = blocks[0];

        for (HuskyLens.Block block : blocks) {
            telemetry.addData("various blocks:", block.toString());
            if (block.width > returnTag.width) {
                returnTag = block;
            }
        }
        // Process block data (e.g., get x-center, y-center, width, height, ID)

        return returnTag;
    }
}




