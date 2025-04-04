
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;
@Autonomous(name="Autonomous Clip", group="8617", preselectTeleOp = "Teleop")
@Disabled
public class AutonomousClip extends AutonomousBase {

    boolean autoRunning = false;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap, true);
        robot.slideLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slideRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("State", "B Dawg 😎");
        telemetry.update();

        while (!isStarted()) {
            robot.clawPos = 0.565;
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if (gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if (robot.clawOpen) { //closes claw
                    robot.clawPos = 0.565;
                    robot.clawOpen = false;
                    robot.clawServo.setPosition(robot.clawPos);
                } else { //opens claw
                    robot.clawPos = 0.685;
                    robot.clawOpen = true;
                    robot.clawServo.setPosition(robot.clawPos);
                }
            } //  gamepad1_r_bumper
            // Pause briefly before looping
            idle();
        } // !isStarted

        mainAutonomous();

        telemetry.addData("Program", "Complete");
        telemetry.update();
    } /* runOp */

    public void
     mainAutonomous () {
        // run to chambers and prep for scoring on high chamber (preload)
        processWrist(0.52, .471, 0);
        processChain(1, -805, 0);
        driveToPosition(23,0,0,1,0.85, DRIVE_TO);

        // slide extension to high chamber and score (preload)
        robot.slideLMotor.setPower(.70);
        robot.slideRMotor.setPower(.70);
        sleep(400);
        processClaw(true);
        sleep(100);
        robot.slideLMotor.setPower(-0.8);
        robot.slideRMotor.setPower(-0.8);
        sleep(500);

        // push first sample to human player
        driveToPosition(23.6,29,-0.2,1,0.85, DRIVE_TO);
        driveToPosition(48.1,27.8 ,.5,1,0.85, DRIVE_TO);
        driveToPosition(42.4,45.3,-90.3,1,0.85, DRIVE_TO);
        driveToPosition(-1.5,45.3,-90.3,1,0.85, DRIVE_TO); // change to Thru

        // push second sample
        driveToPosition(42.4,45.3,-90.3,1,0.85, DRIVE_TO);
        driveToPosition(42.4,54.3,-90.3,1,0.85, DRIVE_TO);
        driveToPosition(-1.5,54.3,-90.3,1,0.85, DRIVE_TO);

        // intake first specimen
        processClaw(true);
        processChain(1, -275, 20);
        driveToPosition(-9.5, 32.8, -179.9, 1, 0.85, DRIVE_TO);
        driveToPosition(-10.8, 32.9, -179.6, 1, 0.85, DRIVE_TO);
        processClaw(false);
        processChain(1, -500, 50);
        processChain(1, -1660, 20);

        // run/score to high chamber (1st specimen)
        driveToPosition(12.4,-8.8,179.7,1,0.85, DRIVE_TO);
        processWrist(.506, .110, 0);
        robot.slideLMotor.setPower(.70);
        robot.slideRMotor.setPower(.70);
        sleep(400);
        processClaw(true);
        sleep(100);
        robot.slideLMotor.setPower(-0.8);
        robot.slideRMotor.setPower(-0.8);
        sleep(500);

        //

        sleep(300000);
    } // mainAutonomous

    //awesome and cool manipulator functions ------_______----_____ :> peter the programming python disapproves
    public void processWrist (double left, double right, int preset) {
        if (preset == 0) { // custom positions
            robot.diffyLServo.setPosition(left);
            robot.diffyRServo.setPosition(right);
        }
        else if (preset == 1) { // chamber scoring
            robot.diffyLPos = 0.506; robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.110; robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        else if (preset == 2) { // basket scoring
            robot.diffyLPos = 0.434;    robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.195;    robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        else if (preset == 3) { // wall intake
            robot.diffyLPos = 0.497; robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.500; robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        else if (preset == 4) { // floor intake
            // TODO: FILL THIS SHIT OUT
            // floor intake is slightly different for each scenario (variance in angle
            // and location) --- a preset for floor intake is pretty unlikely due to this
            // and we should probably continue using preset 0 for all three
        } else {
            telemetry.addData("Diffy Status:","you're an idiot bro what did u put as the diffy preset");
            telemetry.update();
        }
    }

    public void processLift (int leftPos, int rightPos, double powa, int preset) {
        if (preset == 0) { // custom

        }
        else if (preset == 1) { // extend to high basket

        }
        else if (preset == 2) { // extend to low basket

        }
        else if (preset == 3) { // lower slides

        }
        else { //

        }
    }

    public void processChain (double power, int target, int wait) {
        robot.chainMotor.setTargetPosition(target);
        robot.chainMotor.setPower(power);
        sleep(wait);
    }

    public void processClaw (boolean open) {
        if (open) {
            robot.clawServo.setPosition(0.685); //opens claw
            robot.clawOpen = true;
        } else {
            robot.clawServo.setPosition(0.562); //closes claw
            robot.clawOpen = false;
        }
    }


    //lame navigation functions boooooooo >:(

}
