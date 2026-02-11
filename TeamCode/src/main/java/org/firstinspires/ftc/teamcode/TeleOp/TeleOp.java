package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TeleopActionRunner;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FiringArm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        TeleopActionRunner actionRunner = new TeleopActionRunner();
        Robot robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDWALL, false);
        byte firing_pattern = 0;
        boolean fieldRelative = true;
        drivetrain.imu.resetYaw();
        boolean DEBUGMODE = false;
        waitForStart();
        while (!isStopRequested()) {
            DEBUGMODE = gamepad2.right_trigger > .5;
            //DRIVER LOGIC
            double d_x = 0;
            if (gamepad1.right_trigger>.5) {d_x = robot.stereoCamera.findAngleToAprilTag();}
            if (gamepad1.y) {robot.intake.runIntake();}
            if (gamepad1.b) {drivetrain.imu.resetYaw();}
            if (gamepad1.x) {robot.intake.reset();}
            if (gamepad1.a) {robot.intake.reverseIntake();}
            drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_bumper, false, gamepad1.right_bumper, DEBUGMODE);
            //OPERATOR LOGIC

            //TRIGGERS AND BUMPERS
            robot.launcher.FireAtPower(gamepad2.left_trigger);
            if (gamepad2.left_bumper) {robot.firingArm.setFiringState(FiringArm.FiringState.firing);}
            if (gamepad2.right_bumper) {robot.firingArm.setFiringState(FiringArm.FiringState.waiting);}

            // DPAD
            if (gamepad2.dpad_up && !DEBUGMODE) {robot.carousel.rotate();}
            if (gamepad2.dpad_left && !DEBUGMODE) {robot.carousel.SetCarouselState(Carousel.CarouselState.Iblue);}
            if (gamepad2.dpad_right && !DEBUGMODE) {robot.carousel.SetCarouselState(Carousel.CarouselState.Ired);}
            if (gamepad2.dpad_down && !DEBUGMODE) {robot.carousel.SetCarouselState(Carousel.CarouselState.Igreen);}
            //BUTTONS
            if (gamepad2.a && !DEBUGMODE) {robot.launcher.FireAtY(robot.launcher.in_to_m(32), robot.stereoCamera.compute_x_distance());}
            if (gamepad2.b && !DEBUGMODE) {robot.launcher.FireAtPower(.8F);}
            if (gamepad2.x && !DEBUGMODE) {
                Action fireCurrentBallAction = robot.createTrajectoryPlanner()
                        .fireAtBasket()
                        .builder.build();
                Actions.runBlocking(fireCurrentBallAction);
            }
            if (gamepad2.y && !DEBUGMODE) {robot.launcher.FireAtPower(.5F);}

            //DEBUG CONTROLS
            if (gamepad2.dpad_right && DEBUGMODE) {
                // D RIGHT Color sensor
                robot.carousel.ReadColorSensor();}
            if (gamepad2.x && DEBUGMODE) {
                // X reset state to 1
                robot.carousel.SetManualCarouselState(1);}
            if (gamepad2.b && DEBUGMODE) {
                //B state to 0
                robot.carousel.SetManualCarouselState(.2);}
            if (gamepad2.a && DEBUGMODE) {
                // X reset state to 1
                robot.carousel.SetManualCarouselState(0);}
            if (gamepad2.y && DEBUGMODE) {
                //B state to 0
                robot.carousel.SetManualCarouselState(.6);}
            if (gamepad2.dpad_left && DEBUGMODE) {
                //D LEFT heading information
                robot.drive.updatePoseEstimate();
                Pose2d pose = robot.drive.localizer.getPose();
                telemetry.addData("x: ", pose.position.x);
                telemetry.addData("y:", pose.position.y);
                telemetry.addData("heading (deg): ", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();}
            if (gamepad2.dpad_up && DEBUGMODE) {
                // D UP imu info
                Orientation IMU = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                double IMU_1 = IMU.firstAngle ;
                double IMU_2 = IMU.secondAngle ;
                double IMU_3 = IMU.thirdAngle ;
                telemetry.addData("IMU FIRST ANGLE: ", IMU_1);
                telemetry.addData("IMU SECOND ANGLE: ", IMU_2);
                telemetry.addData("IMU THIRD ANGLE: ", IMU_3);}
            if (gamepad2.dpad_down && DEBUGMODE) {
                 // D DOWN camera information
                 telemetry.addData("April Tag: ",robot.leftCamera.find_april_tag());
                 if (robot.leftCamera.find_april_tag() == null) {continue;};
                 double d_0 = robot.stereoCamera.compute_x_distance();
                 double d_1 = Camera.GetDistanceFromArea(robot.leftCamera.getAreaOfAprilTag());
                 double d_2 = Camera.GetDistanceFromArea(robot.rightCamera.getAreaOfAprilTag());
                 telemetry.addData("X Distance Stereo: ", d_0);
                 telemetry.addData("X Distance Area: ", d_1);
                 telemetry.addData("X Distance Area: ", d_2);
                 robot.stereoCamera.getCameraInformation();}


            telemetry.update();
        }
    }

}

