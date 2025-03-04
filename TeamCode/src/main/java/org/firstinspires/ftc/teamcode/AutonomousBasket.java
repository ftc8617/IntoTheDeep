
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Basket", group="8617", preselectTeleOp = "Teleop")
public class AutonomousBasket extends AutonomousBase {

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

    public void mainAutonomous () {
        // run to basket
        driveToPosition(-23.3,-16,44.5,1,0.85, DRIVE_TO);
        // slide extension to high basket
        processChain(1,-1656,320);
        robot.slideLMotor.setPower(.75);
        robot.slideRMotor.setPower(.75);
        sleep(400);
        robot.slideLMotor.setPower(0.02);
        robot.slideRMotor.setPower(0.02);

        // run closer to basket and score preset
        driveToPosition(-27.7, -11.5, 43.2, .9, .85, DRIVE_TO);
        processClaw(true);
        sleep(50);

        // back up and lower slide
        driveToPosition(-23.3,-16,44.5,1,.85, DRIVE_TO);
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);

        // drive to first sample
        sleep(100);
        driveToPosition(-21.8, -23.3, 90.5, 1, .85, DRIVE_TO);
        processClaw(true);
        processWrist(0.291,0.706,0);
        processChain(1,-180,400);

        // intake 1st ground sample
        robot.slideLMotor.setPower(.72);
        robot.slideRMotor.setPower(.72);
        sleep(525);
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        sleep(100);
        processClaw(false);

        // lower slides to prepare to score
        processChain(1,-264,25);
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);

        // run to basket 2nd time
        driveToPosition(-23.3,-16,44.5,1,.85, DRIVE_TO);
        processChain(1,-1656,420);
        robot.slideLMotor.setPower(.75);
        robot.slideRMotor.setPower(.75);
        sleep(350);
        //keep slide afloat
        robot.slideLMotor.setPower(0.01);
        robot.slideRMotor.setPower(0.01);
        processWrist(0,0,2);
        sleep(50);

        // run closer to basket and score
        driveToPosition(-27.7, -11.5, 43.2, 1, .85, DRIVE_TO);
        processClaw(true);
        sleep(50);

        // back up and lower slide
        driveToPosition(-23.3,-16,44.5,1,.85, DRIVE_TO);
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        sleep(200);

        //next intake! (2nd)
        driveToPosition(-32,-22.6,87.7,1,.85,DRIVE_TO);
        processWrist(0.262,0.736,0); //
        processChain(1,-219,450); //
        robot.slideLMotor.setPower(.70);
        robot.slideRMotor.setPower(.70);
        sleep(580);
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        sleep(150);
        processClaw(false);
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        sleep(150);

        // next score - 2nd sample (3rd score!)
        driveToPosition(-23.3,-16,44.5,1,.85, DRIVE_TO);
        processChain(1,-1656,390);
        robot.slideLMotor.setPower(0.73);
        robot.slideRMotor.setPower(0.73);
        sleep(190);
        // set to 0 so slide stops
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        sleep(10);
        robot.slideLMotor.setPower(0.01);
        robot.slideRMotor.setPower(0.01);
        processWrist(0,0,2);
        sleep(50);

        // run closer to basket and score (2nd sample)
        driveToPosition(-27.7, -11.5, 43.2, 1, .85, DRIVE_TO);
        processClaw(true);
        sleep(50);

        // back up and lower slide (2nd sample)
        driveToPosition(-23.3,-16,44.5,1,.85, DRIVE_TO);
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);

        sleep(150);

        // 3rd intake! - TODO: FIX THIS AT SOME POINT IN TIME TO BE ACTUALLY CONSISTENT PLS.
        driveToPosition(-37.5,-27.5,106.6,.9,.80, DRIVE_TO);
        processChain(1.0,-200,580); //
        processWrist(0.196,0.762,0); //
        robot.slideLMotor.setPower(.68);
        robot.slideRMotor.setPower(.68);
        sleep(510);
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        sleep(150);
        processClaw(false);
        sleep(50);

        processWrist(0,0,3);
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        sleep(175);

        // 4th score!!!!!!!! (3rd sample)
        driveToPosition(-23.3,-16,44.5,1,.85, DRIVE_TO);
        processChain(1,-1656,450);
        robot.slideLMotor.setPower(1);
        robot.slideRMotor.setPower(1);
        sleep(350);
        // set to 0 so slide stops
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        processWrist(0,0,2);
        sleep(25);

        // run closer to basket and score (3rd sample)
        driveToPosition(-27.7, -11.5, 43.2, 1, .65, DRIVE_TO);
        processClaw(true);
        sleep(50 );

        // get everything tight for init in tele
        driveToPosition(-23.3,-16,44.5,1,.85, DRIVE_TO);
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        processChain(1,0,0);
        sleep(30000000);

    } // mainAutonomous

    //awesome and cool manipulator functions ------_______----_____ :> peter the programming python approves
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
