/* FTC Team 8617 - Version 1.0 (03/17/2025)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Right: clip 5 (83 pts)", group="8617", preselectTeleOp = "Teleop")
//@Disabled
public class AutonomousRight5 extends AutonomousBase {
    boolean debugMode = true;

    double pos_y=0, pos_x=0, pos_angle=0.0;  // Allows us to specify movement INCREMENTALLY, not ABSOLUTE

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap,true);


        telemetry.addData("State", "B Dawg ??");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            robot.clawPos = 0.565;
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if (gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if (robot.clawOpen) { // closes claw
                    robot.clawPos = 0.565;
                    robot.clawOpen = false;
                    robot.clawServo.setPosition(robot.clawPos);
                } else { // opens claw
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

    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    /* Autonomous Right:                                                                          */
    /*   1 Starting Point                                                                         */
    /*   2 Hang pre-load specimen at submersible                                                  */
    /*   3 Herd samples from spike marks (left/center/wall)                                       */
    /*   4 Grab clipped specimen from observation zone wall                                       */
    /*   5 Hang specimen on high rung (repeat steps 4 & 5)                                        */
    /*   6 Park in observation zone                                                              */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {
        int specimensHooked  = 0;
        int specimensGrabbed = 0;

        // Score the preloaded SPECIMEN
        hookSpecimenOnBarFWD( specimensHooked++ );

        // Go herd all 3 samples into observation zone
        herdSamples();

        // Get the 2nd preload that should already be on the wall
        grabSpecimenFromWall( specimensGrabbed++ );
        hookSpecimenOnBarBWD( specimensHooked++ );

        // Score first of the three herded samples
        grabSpecimenFromWall( specimensGrabbed++ );
        hookSpecimenOnBarBWD( specimensHooked++ );

        // Score second of the three herded samples
        grabSpecimenFromWall( specimensGrabbed++ );
        hookSpecimenOnBarBWD( specimensHooked++ );

        // Score third of the three herded samples
        grabSpecimenFromWall( specimensGrabbed++ );
        hookSpecimenOnBarBWD( specimensHooked++ );

        // Park in observation zone for for 3pts (if time allows)
        parkInObservation();
        
        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void hookSpecimenOnBarFWD(int specimenNumber ) {

        telemetry.addData("Motion", "Move to submersible");
        telemetry.update();

        // If this is the initial preloaded specimen, inch forward away from the wall
        // (raising the lift while against the wall will cause lift motor to hit rear wall)
        if( opModeIsActive() ) {
            // Move 3 inches away from field wall before we begin to lift our slides
            driveToPosition( 3.00, 0.00, 0.00, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            pos_x = -6.25; // hang initial specimen 6.25 inches to the left of starting position
        } // opModeIsActive

        // Drive toward submersible
        if( opModeIsActive() ) {
            // Start tilting and extending the arm, and positioning the specimen
//          autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN1_DEG, 1.0);
//          autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO1);
            // Drive to the scoring position next to the submersible
            driveToPosition( 16.2, (pos_x+2.2), 0.00, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
//          robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR1);
//          robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR1);
            pos_y = 27.80 + (specimenNumber * 0.25);  // specimenNumber doesn't matter, as only use once
            driveToPosition( pos_y, pos_x, 0.00, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_TO );
            robot.driveTrainMotorsZero();  // make double sure we're stopped
            // If we drive to the submersible faster than the arm moves, wait for the arm
            //sleep(100);
        } // opModeIsActive

        // Rotate arm, viper slide, and claw down to clip the specimen
        if( opModeIsActive() ) {
//          autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN2_DEG,0.80 );
//          autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO2);
//          robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR2);
//          robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR2);
//          sleep( 1200 ); //while( autoTiltMotorMoving() || autoViperMotorMoving());
            // release the specimen
//          robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        } // opModeIsActive

        // Retract the arm for driving as we pivot away from the submersible
        if( opModeIsActive() ) {
//          autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_ZERO);
            // Back away in preparation of herding
            driveToPosition(26.5, 0.0, 90.0, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU);
        } // opModeIsActivee

        // Now that we're clear from the submersible, rotate arm down and store claw
        if( opModeIsActive() ) {
//          robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_INIT);
//          robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_INIT);
//          autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL_DEG, 0.80);
        } // opModeIsActive

    }  // hookSpecimenOnBarFWD

    /*--------------------------------------------------------------------------------------------*/
    private void hookSpecimenOnBarBWD(int specimenNumber ) {

        telemetry.addData("Motion", "Move to submersible");
        telemetry.update();

        // Drive toward submersible
        if( opModeIsActive() ) {
            // Start tilting and extending the arm, and positioning the specimen
//          autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN3_DEG, 1.0);
//          autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO3);
            // shift left 3" each time we hang a new specimen
            pos_x = -6.80 - (3.0 * specimenNumber);
            // Drive partway in Y and X toward the scoring position next to the submersible
            driveToPosition( 18.2, (pos_x+10.0), 180.0, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
//          robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR3);
//          robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR3);
            // Finish the drive to the submersible bar
            pos_y = 26.40 + (specimenNumber * 0.15);
            driveToPosition( pos_y, pos_x, 180.0, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_TO );
            robot.driveTrainMotorsZero();  // make double sure we're stopped
        } // opModeIsActive

        // Retract slide to clip the specimen, then release the claw
        if( opModeIsActive() ) {
//          autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO4);
//          sleep( 800 );
            // release the specimen
//          robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
        } // opModeIsActive

        // Fully retract arm for driving as we pivot away from the submersible
        if( opModeIsActive() ) {
//          autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_ZERO);
//          robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_INIT);
//          robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_INIT);
//          autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL_DEG, 0.80);
        } // opModeIsActivee

    }  // hookSpecimenOnBarBWD

    /*--------------------------------------------------------------------------------------------*/
    private void herdSamples() {
        // Do we herd the first sample on the spike marks?
        if( opModeIsActive() ) {
            // Navigate around the corner of the submersible
            driveToPosition( 26.7, 10.5, 112.0, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            driveToPosition( 30.3, 18.4, 137.0, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            driveToPosition( 34.0, 20.0, 180.00, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            // Align for the first sample
            pos_y=46.5; pos_x=23.7; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            pos_x+=7.0; // 7" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            pos_y-=27.5; // 27.5" back toward observation zone
            herdForwardQuickly( pos_y, pos_x, pos_angle, DRIVE_SPEED_100 );
        } // opModeIsActive
        // What about the 2nd sample?
        if( opModeIsActive() ) {
            pos_y=45.0; pos_x=32.5; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            pos_x+=7.0; // 7" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            pos_y-=27.5; // 27.5" back toward observation zone
            herdForwardQuickly( pos_y, pos_x, pos_angle, DRIVE_SPEED_100 );
        } // opModeIsActive
        // What about the 3rd one against the wall?
        if( opModeIsActive() ) {
            pos_y=46.0; pos_x=40.5; pos_angle=180.0; // start at this absolute location
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            pos_x+=1.5; // 1.5" toward wall/samples
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            timeDriveStrafe(DRIVE_SPEED_20,1000); // ensure we slowly align to the wall
            // What does odometry report as our new X,Y location? (we're aligned to wall, so by
            // definition we're at 180deg, even if the initial alignment was off a degree or two
            pos_y=robotGlobalYCoordinatePosition;
            pos_x=robotGlobalXCoordinatePosition;
            robotOrientationRadians = Math.toRadians(180.0);
            // Drive away from the wall in a DIAGONAL FORWARD movement (driving  sideways away
            // from the wall might leave our magnetic sign attached to the field wall!
            pos_y -= 5.0;
            pos_x -= 1.0;
           // pos_angle=175.0;  // angle the bot away from the wall as we herd the final sample
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            // Go fast to the edge of the observation zone
            pos_y =  17.0;
            pos_x -= 2.5;  // end 2" away from the wall
            driveToPosition( pos_y, pos_x, pos_angle, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            // ease into the observation zone (in case we hit the wall, or another robot)
            timeDriveStraight(DRIVE_SPEED_20,1000);  // this stops all motors
            // NOTE: this ending position also counts as PARKED
        } // opModeIsActive

    } // herdSamples

    /*--------------------------------------------------------------------------------------------*/
    private void grabSpecimenFromWall( int specimenNumber ) {

        // Prepare arm for grabbing specimens from wall, and move to initial wall-collect position (quickly)
        if( opModeIsActive() ) {
//          autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL0_DEG, 1.0);
//          autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_WALL0);
//          robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_WALL0);
//          robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_WALL0);
//          robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_OPEN_WIDE );
            if( specimenNumber == 0 ) {
               // Approach along x-axis from herding spike marks...
               driveToPosition( 8.0, 28.0, 180, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            } else {
               // Approach along y-axis from hooking at submersible...
               driveToPosition( 9.5, 13.2, 180, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_THRU );
            }
        } // opModeIsActive

        // Drive to the final wall-collect position (slowly)
        if( opModeIsActive() ) {
            driveToPosition( 4.6, 18.6, 180, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_TO );
//          robot.clawStateSet( Hardware2025Bot.clawStateEnum.CLAW_CLOSED );
//          sleep(350); // allow claw to close (350msec)
        } // opModeIsActive

        // Lift the specimen off the field wall
        if( opModeIsActive() ) {
//          autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL1_DEG, 1.0);
//          autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_WALL1);
//          robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_WALL1);
//          robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_WALL1);
//          sleep(600); // allow arm to lift above the wall (600 msec)
        } // opModeIsActive

    } // grabSpecimenFromWall

    /*--------------------------------------------------------------------------------------------*/
    private void parkInObservation() {
        if( opModeIsActive() ) {
            // Park in far corner of observation zone
            driveToPosition(8.0, 30.0,  90.0, DRIVE_SPEED_100, TURN_SPEED_100, DRIVE_TO);
        }
    } // parkInObservation

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
            robot.clawServo.setPosition(0.685); //opens claw
            robot.clawOpen = true;
        } else {
            robot.clawServo.setPosition(0.562); //closes claw
            robot.clawOpen = false;
        }
    }
} /* AutonomousRight5 */
