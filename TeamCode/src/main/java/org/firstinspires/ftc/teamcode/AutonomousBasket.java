
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Basket", group="8617", preselectTeleOp = "Teleop")
public class AutonomousBasket extends AutonomousBase {
    boolean debugMode = true;

    boolean done = false;
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

        // determines if we print telemetry values during lift operations
        while (!isStarted()) {
            robot.clawPos = 0.87;
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if (gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if (robot.clawOpen) { // closes claw
                    robot.clawPos = 0.870;
                    robot.clawOpen = false;
                    robot.clawServo.setPosition(robot.clawPos);
                } else { // opens claw
                    robot.clawPos = 0.97;
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
        // score preset
        scoreHighBasket();

        // collect first intake
        intake1stSpike();
        scoreHighBasket();

        // collect second intake
        intake2ndSpike();
        scoreHighBasket();

        // collect third intake
        intake3rdSpike();
        done = true;
        scoreHighBasket();

        robot.driveTrainMotorsZero();
        sleep(30000000);

    } // mainAutonomous

    //awesome and cool manipulator functions ------_______----_____ :> peter the programming python approves
    public void processWrist (double left, double right, int preset) {
        if (preset == 0) { // custom positions
            robot.diffyLServo.setPosition(left);
            robot.diffyRServo.setPosition(right);
        }
        else if (preset == 1) { // chamber scoring
            robot.diffyLPos = 0.026; robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.570; robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        else if (preset == 2) { // basket scoring
            robot.diffyLPos = 0.150;    robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.457;    robot.diffyRServo.setPosition(robot.diffyRPos);
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

    public void processLift (int targetPos, double speedMultiplier, double initialSpeedMultiplier, double hold) {

        robot.autoSlidePositionStart(targetPos);

        while (opModeIsActive() && robot.slideMotorBusy ){
            robot.readBulkData();
            robot.autoSlidePositionUpdate(speedMultiplier, initialSpeedMultiplier);

            if(debugMode) {
                telemetry.addData("Slide Busy", robot.slideMotorBusy);
                telemetry.addData("Slide Position", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos, robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower());
                telemetry.addData("Slide Error", "%d %d cts", robot.positionLError, robot.positionRError);
                telemetry.update();
            }
        }

        robot.slideLMotor.setPower(hold);
        robot.slideRMotor.setPower(hold);
    }

    public void processChain (double power, int target, int wait) {
        robot.chainMotor.setTargetPosition(target);
        robot.chainMotor.setPower(power);
        sleep(wait);
    }

    public void processClaw (boolean open) {
        if (open) {
            robot.clawServo.setPosition(0.97); //opens claw
            robot.clawOpen = true;
        } else {
            robot.clawServo.setPosition(0.870); //closes claw
            robot.clawOpen = false;
        }
    }

    public void scoreHighBasket () {
        processChain(1, -1655, 0);
        processWrist(0.434, 0.195, 2);
        driveToPosition(-25, -6.9, 45, 1, 1, DRIVE_TO);
        robot.slideLMotor.setPower(1);
        robot.slideRMotor.setPower(1);
        processLift(2200,2,2.5,0.07);
        processWrist(0.506, 0.110, 1);
        veryErrorProne(-26, -5.9, 45, .5);
        sleep(100);
        processClaw(true);
        sleep(75);
        if (done) {
            //processChain(.75, -273, 0);
            ascentPark();
        }
        else {
            driveToPosition(-25, -6.9, 45, 1, 1, DRIVE_THRU);
        }
        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        processWrist(0.497, 0.5, 0);
    } // scoreHighBasket

    public void intake1stSpike () {
        driveToPosition(-17.1,-13.9,86.6,1,1, DRIVE_TO);
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        processChain(1,-200, 50);
        processLift(1080,2,3,0);
        sleep(100);
        processWrist(0.646,0.366,0);
        sleep(100);
        processClaw(false);
        sleep(70);
        processChain(1,-273,0);

        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        sleep(250);
    }

    public void intake2ndSpike () {
        driveToPosition(-26.65,-13.9, 90,1,1, DRIVE_TO);
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        processChain(1,-200, 50);
        processLift(1100,2,3,0);
        sleep(100);
        processWrist(0.646,0.366,0);
        sleep(100);
        processClaw(false);
        sleep(70);
        processChain(1,-273,0);

        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        sleep(200);
    }

    public void intake3rdSpike () {
        driveToPosition(-28.7,-15.7,107.5,1,1, DRIVE_TO);
        robot.slideLMotor.setPower(0);
        robot.slideRMotor.setPower(0);
        processChain(1,-235, 50);
        processLift(1180,2,3,0);
        sleep(100);
        processWrist(0.666,0.336,0);
        sleep(100);
        processChain(1,-190, 50);
        sleep(100);
        processClaw(false);
        sleep(70);
        processChain(1,-270,0);

        robot.slideLMotor.setPower(-1);
        robot.slideRMotor.setPower(-1);
        done = true;
        sleep(250);
    }

    public void ascentPark () {
        driveToPosition(-25, -6.9, 45, 1, 1, DRIVE_TO);
    }
    //lame navigation functions boooooooo >:(

}
