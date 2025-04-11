
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Autonomous Test Zawg2", group="8617", preselectTeleOp = "Teleop")
@Disabled
public class AutonomousTestZawg2 extends AutonomousBase {
    HardwareZawg2 robot = new HardwareZawg2();

    boolean autoRunning = false;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap, true);

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
            // Pause briefly before looping
            idle();
        } // !isStarted

        mainAutonomous();

        //telemetry.addData("Program", "Complete");
        //telemetry.update();
        while(opModeIsActive()) {
            robot.readBulkData();
            telemetry.addData("Slide", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos,  robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower() );
            telemetry.update();
        }
        sleep(3000000);
    } /* runOp */

    public void mainAutonomous () {
        processChain(1,-273,500);

        robot.autoSlidePositionStart(1000 );

        while (opModeIsActive() && robot.slideMotorBusy ){
            robot.readBulkData();
            //robot.autoSlidePositionUpdate();
            telemetry.addData("Slide Busy", robot.slideMotorBusy);
            telemetry.addData("Slide pos", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos,  robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower() );
            telemetry.addData("Slide Error", "%d %d cts", robot.positionLError,  robot.positionRError);
            telemetry.update();
        }


        sleep(5000);

        robot.autoSlidePositionStart(800 );

        while (opModeIsActive() && robot.slideMotorBusy ){
            robot.readBulkData();
            //robot.autoSlidePositionUpdate();
            telemetry.addData("Slide Busy", robot.slideMotorBusy);
            telemetry.addData("Slide pos", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos,  robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower() );
            telemetry.addData("Slide Error", "%d %d cts", robot.positionLError,  robot.positionRError);
            telemetry.update();
        }

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
