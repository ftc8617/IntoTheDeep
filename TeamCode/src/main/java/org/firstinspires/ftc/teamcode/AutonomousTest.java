
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Test", group="8617", preselectTeleOp = "Teleop")
public class AutonomousTest extends AutonomousBase {

    boolean autoRunning = false;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap, true);
        robot.slideLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slideLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.slideLMotor.setTargetPosition(robot.slideLMotorPos);
        robot.slideRMotor.setTargetPosition(robot.slideRMotorPos);
        robot.slideLMotor.setPower(0.5);
        robot.slideRMotor.setPower(0.5);
//        robot.diffyLPos = 0.497; robot.diffyLServo.setPosition(robot.diffyLPos);
//        robot.diffyRPos = 0.500; robot.diffyRServo.setPosition(robot.diffyRPos);

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
            telemetry.addData("Slide", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos,  robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower() );
            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        mainAutonomous();

        //telemetry.addData("Program", "Complete");
        //telemetry.update();
        while(opModeIsActive()) {
            robot.readBulkData();
            telemetry.addData("Slide tgt", "%d %d cts ", robot.slideLMotor.getTargetPosition(),  robot.slideRMotor.getTargetPosition() );
            telemetry.addData("Slide pos", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos,  robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower() );
            telemetry.update();
        }
        sleep(3000000);
    } /* runOp */

    public void mainAutonomous () {
       // herdForwardQuickly( 30, 0, 0, 1 );
    processChain(1,-1600,500);
        robot.slideLMotor.setTargetPosition(-810);
        robot.slideRMotor.setTargetPosition(-810);

//        sleep(5000);
//        robot.slideLMotor.setTargetPosition(-810);
//        robot.slideRMotor.setTargetPosition(-810);
//        while (robot.slideRMotorPos>-800 && opModeIsActive()){
//            robot.readBulkData();
//            telemetry.addData("Slide ", "%d %d counts", robot.slideLMotorPos,  robot.slideRMotorPos );
//            telemetry.addData("Chain ", "%d counts", robot.chainMotorPos );
//            telemetry.update();
//        }
//        robot.slideLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.slideRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        robot.slideLMotor.setPower(0);
//        robot.slideRMotor.setPower(0);
//        telemetry.addData("while done", "%d counts", 2);
//        telemetry.addData("Slide ", "%d %d counts", robot.slideLMotorPos,  robot.slideRMotorPos );
//        telemetry.addData("Chain ", "%d counts", robot.chainMotorPos );
//        telemetry.update();







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
