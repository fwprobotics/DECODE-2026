package org.firstinspires.ftc.teamcode.pipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;

import java.util.ArrayList;

public class ApriltagDetection implements VisionProcessor {
    private static final double FEET_PER_METER = 3.28;
    // STATIC CONSTANTS


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }



    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }



//    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    // UNITS ARE METERS
    public static double TAG_SIZE = 0.166;

    // instance variables

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    double tagsizeX = TAG_SIZE;
    double tagsizeY = TAG_SIZE;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();


    public boolean waiting = true;
    Telemetry telemetry;

    public ApriltagDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public AprilTagDetection getLastDetection (){
        return detections.get(-1);
    }

    @Override
    public void finalize()
    {
        // Delete the native context we created in the init() function
        AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        Core.rotate(input, input, Core.ROTATE_180);
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        //     labelPixels(input, purple_contours, COLORS.PURPLE);
        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, TAG_SIZE, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        if (detections.size() > 0) {
            AprilTagDetection d = detections.get(0);
            Imgproc.circle(input, d.corners[2], 5, new Scalar(255, 0, 0, 255), -1);
            Imgproc.circle(input, d.corners[1], 5, new Scalar(0, 255, 0, 255), -1);
            Imgproc.circle(input, d.corners[3], 5, new Scalar(0, 0, 255, 255), -1);
            telemetry.addLine("corner 2: " + d.corners[2].toString());
            telemetry.addLine("corner 1: " + d.corners[1].toString());
            double inch_to_pixel = Math.sqrt(Math.pow(d.corners[1].x - d.corners[2].x, 2) + Math.pow(d.corners[1].y - d.corners[2].y, 2)) / 2;
            telemetry.addLine("size " + inch_to_pixel);


            Orientation rot = Orientation.getOrientation(d.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

            double tanofangtle = (d.corners[2].y - d.corners[1].y) / (d.corners[2].x - d.corners[1].x);
            double angle = Math.toDegrees(Math.atan(tanofangtle));
            if (angle < 0) {
                angle += 90;
            } else {
                angle -= 90;
            }
            telemetry.addLine("angle " + angle);
            Point rotPoint = new Point(input.cols() / 2.0,
                    input.rows() / 2.0);
            // Mat rotMat = Imgproc.getRotationMatrix2D(rotPoint, -angle, 1);
            // Imgproc.warpAffine(input, input, rotMat, input.size(),
            // Imgproc.WARP_INVERSE_MAP);
            Rect crop = new Rect(0, 0, 0, 0);
            if (d.id == 1 || d.id == 4) {
                int xStart = (int) ((int) d.corners[3].x - (3 * inch_to_pixel));
                int yStart = 0;
                int xOver = (int) Math.max(0, Math.min(input.size().width - xStart, (int) (20 * inch_to_pixel)));
                int yOver = (int) Math.max(0, Math.min(input.size().height - yStart, (int) ((int) d.corners[1].y - (3 * inch_to_pixel))));
                crop = new Rect(xStart, yStart, xOver, yOver);
            } else if (d.id == 2 || d.id == 5) {
                int xStart = 0;
                int yStart = 0;
                int xOver = (int) Math.max(0, Math.min(input.size().width - xStart, (int) ((int) d.corners[1].x + (4 * inch_to_pixel))));
                int yOver = (int) Math.max(0, Math.min(input.size().height - yStart, (int) ((int) d.corners[1].y - (2 * inch_to_pixel))));
                crop = new Rect(xStart, yStart, xOver, yOver);
            } else {
                int xStart = (int) (d.corners[1].x - (5 * inch_to_pixel));
                int yStart = 50;
                int xOver = (int) Math.max(0, Math.min(input.size().width - xStart, ((int) +(20 * inch_to_pixel))));
                int yOver = (int) Math.max(0, Math.min(input.size().height - yStart, (int) ((int) d.corners[1].y - (6 * inch_to_pixel))));
                crop = new Rect(xStart, yStart, xOver, yOver);
            }
            input = new Mat(input, crop);
        }
        return  input;
    }


    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }




}