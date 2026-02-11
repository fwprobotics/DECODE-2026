package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.FiringArm;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@Autonomous
public class MeetThreeAuto extends LinearOpMode {

    int sequence;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.AutoPos autoPos = Robot.AutoPos.BLUEWALL;
        while (!gamepad1.x) {
            if (gamepad1.dpad_down) {
                autoPos = Robot.AutoPos.REDWALL;
            } else if (gamepad1.dpad_up) {
                autoPos = Robot.AutoPos.BLUEWALL;
            }
            if (gamepad1.dpad_left) {
                autoPos = Robot.AutoPos.REDBASKET;
            } else if (gamepad1.dpad_right) {
                autoPos = Robot.AutoPos.BLUEBASKET;
            }
            telemetry.addData("CURRENT POSITION: ", autoPos);
            telemetry.update();
        }
        telemetry.addData("READY FOR: ", autoPos);
        telemetry.update();
        Robot robot = new Robot(hardwareMap, telemetry, autoPos, false);
        int guess = robot.stereoCamera.getSequence();
        FieldTrajectoryPlanner traj = robot.createTrajectoryPlanner()
                .stepToShot();
        if (guess> 10) {
            sequence = guess % 10;
            telemetry.addData("SEQUENCE BEING USED: ", sequence);
            telemetry.update();
            traj = traj.fireWholeMagazineInSequence(sequence % 10);
        }
        else {
            traj = traj.determineThenFire();
        }
        Actions.runBlocking(robot.firingArm.setFiringStateAction(FiringArm.FiringState.waiting));

        traj = traj.lineWithBackWall();
        Action autoAction = traj.builder.build();

        waitForStart();

        Actions.runBlocking(autoAction);
    }
}
