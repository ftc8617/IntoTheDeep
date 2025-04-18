/* FTC Team 8617 - Version 1.0 (10/01/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TeleOp (with test modes).
 */
@TeleOp(name="Teleop", group="8617")
public class Teleop extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;
    boolean gamepad1_square_last,     gamepad1_square_now     = false;
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  //
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  //
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  //
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  //
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  //
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  //
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  //
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  //
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  //
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  //
    boolean gamepad2_touchpad_last,   gamepad2_touchpad_now   = false;  //
    boolean gamepad2_share_last,      gamepad2_share_now      = false;  //

    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  rearLeft, rearRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    boolean controlMultSegLinear = true;

    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    elapsedTime, elapsedHz;

    double curX, curY, curAngle;
    double minX=0.0, maxX=0.0, minY=0.0, maxY=0.0;

    //rumble settings
    Gamepad.RumbleEffect shortRumble;
    Gamepad.RumbleEffect leftRumble;

    Gamepad.RumbleEffect leftDoubleRumble;
    Gamepad.RumbleEffect rightRumble;
    Gamepad.RumbleEffect rightDoubleRumble;

    /* Declare OpMode members. */
    HardwareZawg2 robot = new HardwareZawg2();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        //creating rumble effects
        shortRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1,1,200) //rumble both sides for 200ms
                .build();

        leftRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1,0,200)
                .build();
        leftDoubleRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1,0,200)
                .addStep(0,0,100)
                .addStep(1,0,200)
                .build();

        rightRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0,1,200)
                .build();
        rightDoubleRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(0,1,200)
                .addStep(0,0,100)
                .addStep(0,1,200)
                .build();

        // Initialize robot hardware (not autonomous mode)
        robot.init(hardwareMap,false);


        // Preload and initialize
        robot.chainMotor.setTargetPosition(robot.chainMotorPos);

        robot.slideLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slideRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "austen james richardson");
        telemetry.addLine("Press X (cross) to reset encoders");
        telemetry.addLine("(to run Teleop without Auto first)");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Normally autonomous resets encoders.  Do we need to for teleop??
            if( gamepad1_cross_now && !gamepad1_cross_last) {
                robot.resetEncoders();
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Refresh gamepad button status
            captureGamepad1Buttons();
            captureGamepad2Buttons();

            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            robot.odom.update();
            Pose2D pos = robot.odom.getPosition();  // x,y pos in inch; heading in degrees
            curX     = pos.getX(DistanceUnit.INCH);
            curY     = pos.getY(DistanceUnit.INCH);
            curAngle = pos.getHeading(AngleUnit.DEGREES);
            String posStr = String.format(Locale.US, "{X,Y: %.1f, %.1f in  H: %.1f deg}", curX, curY, curAngle);
            telemetry.addData("Position", posStr);
            //========== PINPOINT ODOMETRY CALIBRATION CODE (x/y offsets) =======
          if(curX<minX) {minX=curX;} if(curX>maxX){maxX=curX;}
          if(curY<minY) {minY=curY;} if(curY>maxY){maxY=curY;}
          double x_radius_mm = 25.4*(maxX-minX)/2.0;
          double y_radius_mm = 25.4*(maxY-minY)/2.0;
          telemetry.addData("Odo Circle","x-radius=%.2f y-radius=%.2f [mm]", x_radius_mm, y_radius_mm);
            //===================================================
                telemetry.addData("Odo Status", robot.odom.getDeviceStatus());
            telemetry.addData(" "," ");

            if( processDpadDriveMode() == false ) {
                // Control based on joystick; report the sensed values
                telemetry.addData("Joystick", "x=%.3f, y=%.3f spin=%.3f",
                        gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x );
                processStandardDriveMode();
            } // processDpadDriveMode

            encoderReset();
            processLift();
            chainMotorMovement();
            diffyMovement();
            clawMovement();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            elapsedHz    =  1000.0 / elapsedTime;

            // Update telemetry data
            telemetry.addData("Front", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    frontLeft, robot.frontLeftMotorVel, frontRight, robot.frontRightMotorVel );
            telemetry.addData("Rear ", "%.2f (%.0f cts/sec) %.2f (%.0f cts/sec)",
                    rearLeft,  robot.rearLeftMotorVel,  rearRight,  robot.rearRightMotorVel );
            telemetry.addData("Front", "%d %d counts", robot.frontLeftMotorPos, robot.frontRightMotorPos );
            telemetry.addData("Back ", "%d %d counts", robot.rearLeftMotorPos,  robot.rearRightMotorPos );
//          telemetry.addData("Slide ", "%d %d counts", robot.slideLMotorPos,  robot.slideRMotorPos );
            telemetry.addData("Slide ", "%d %d cts (%.4f %.4f)", robot.slideLMotorPos,  robot.slideRMotorPos, robot.slideLMotor.getPower(), robot.slideRMotor.getPower() );
//          telemetry.addData("Chain ", "%d counts", robot.chainMotorPos );
            telemetry.addData("Chain ", "%d counts (%.1f)", robot.chainMotorPos, robot.chainMotor.getPower() );
            telemetry.addData("Claw ", "%.3f counts", robot.clawPos );
            telemetry.addData("Diffy ", "%.3f %.3f counts", robot.diffyLPos, robot.diffyRPos );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
            telemetry.addData("version","103");
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_triangle_last   = gamepad1_triangle_now;    gamepad1_triangle_now   = gamepad1.triangle;
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_square_last     = gamepad1_square_now;      gamepad1_square_now     = gamepad1.square;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
    } // captureGamepad1Buttons

    /*---------------------------------------------------------------------------------*/
    void captureGamepad2Buttons() {
        gamepad2_triangle_last   = gamepad2_triangle_now;    gamepad2_triangle_now   = gamepad2.triangle;
        gamepad2_circle_last     = gamepad2_circle_now;      gamepad2_circle_now     = gamepad2.circle;
        gamepad2_cross_last      = gamepad2_cross_now;       gamepad2_cross_now      = gamepad2.cross;
        gamepad2_square_last     = gamepad2_square_now;      gamepad2_square_now     = gamepad2.square;
        gamepad2_dpad_up_last    = gamepad2_dpad_up_now;     gamepad2_dpad_up_now    = gamepad2.dpad_up;
        gamepad2_dpad_down_last  = gamepad2_dpad_down_now;   gamepad2_dpad_down_now  = gamepad2.dpad_down;
        gamepad2_dpad_left_last  = gamepad2_dpad_left_now;   gamepad2_dpad_left_now  = gamepad2.dpad_left;
        gamepad2_dpad_right_last = gamepad2_dpad_right_now;  gamepad2_dpad_right_now = gamepad2.dpad_right;
        gamepad2_l_bumper_last   = gamepad2_l_bumper_now;    gamepad2_l_bumper_now   = gamepad2.left_bumper;
        gamepad2_r_bumper_last   = gamepad2_r_bumper_now;    gamepad2_r_bumper_now   = gamepad2.right_bumper;
//      gamepad2_touchpad_last   = gamepad2_touchpad_now;    gamepad2_touchpad_now   = gamepad2.touchpad;
//      gamepad2_share_last      = gamepad2_share_now;       gamepad2_share_now      = gamepad2.share;
    } // captureGamepad2Buttons

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineControlSpeed = 0.20;
        boolean dPadMode = true;
        // Only process 1 Dpad button at a time
        if( gamepad1.dpad_up ) {
            telemetry.addData("Dpad","FORWARD");
            frontLeft  = fineControlSpeed;
            frontRight = fineControlSpeed;
            rearLeft   = fineControlSpeed;
            rearRight  = fineControlSpeed;
        }
        else if( gamepad1.dpad_down ) {
            telemetry.addData("Dpad","BACKWARD");
            frontLeft  = -fineControlSpeed;
            frontRight = -fineControlSpeed;
            rearLeft   = -fineControlSpeed;
            rearRight  = -fineControlSpeed;
        }
        else if( gamepad1.dpad_left ) {
            telemetry.addData("Dpad","LEFT");
            frontLeft  = -fineControlSpeed;
            frontRight =  fineControlSpeed;
            rearLeft   =  fineControlSpeed;
            rearRight  = -fineControlSpeed;
        }
        else if( gamepad1.dpad_right ) {
            telemetry.addData("Dpad","RIGHT");
            frontLeft  =  fineControlSpeed;
            frontRight = -fineControlSpeed;
            rearLeft   = -fineControlSpeed;
            rearRight  =  fineControlSpeed;
        }
        else {
            dPadMode = false;
        }
        if( dPadMode ) {
            robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight);
        }
        return dPadMode;
    } // processDpadDriveMode

    private double minThreshold( double valueIn ) {
        double valueOut;

        //========= NO/MINIMAL JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.02 ) {
            valueOut = 0.0;
        }
        else {
            valueOut = valueIn;
        }
        return valueOut;
    } // minThreshold

    private double multSegLinearRot( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.33 ) {                      // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.0650;   // 0.02=0.070  0.33=0.1475
            }
            else if( valueIn < 0.60 ) {
                valueOut = (0.50 * valueIn) - 0.0175;   // 0.33=0.1475  0.60=0.2825
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.1675;   // 0.60=0.2825  0.90=0.5075
            }
            else
                valueOut = (6.00 * valueIn) - 4.8925;   // 0.90=0.5075  1.00=1.1075 (clipped!)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.33 ) {
                valueOut = (0.25 * valueIn) - 0.0650;
            }
            else if( valueIn > -0.60 ) {
                valueOut = (0.50 * valueIn) + 0.0175;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.1675;
            }
            else
                valueOut = (6.00 * valueIn) + 4.8925;
        }

        return valueOut/2.0;
    } // multSegLinearRot

    private double multSegLinearXY( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.50 ) {                       // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.040;     // 0.01=0.0425   0.50=0.1650
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.210;     // 0.50=0.1650   0.90=0.4650
            }
            else
                valueOut = (8.0 * valueIn) - 6.735;      // 0.90=0.4650   1.00=1.265 (clipped)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.50 ) {
                valueOut = (0.25 * valueIn) - 0.040;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.210;
            }
            else
                valueOut = (8.0 * valueIn) + 6.735;
        }

        return valueOut;
    } // multSegLinearXY

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Standard Mecanum-wheel drive control (no dependence on gyro!)         */
    /*---------------------------------------------------------------------------------*/
    void processStandardDriveMode() {
        // new awesone roarbots code

        // Retrieve X/Y and ROTATION joystick input
        if( controlMultSegLinear ) {
            yTranslation = multSegLinearXY( .94 * -gamepad1.left_stick_y );
            xTranslation = multSegLinearXY(  .94 * gamepad1.left_stick_x );
            rotation     = multSegLinearRot( 1  * -gamepad1.right_stick_x );

        }
        else {
            yTranslation = -gamepad1.left_stick_y * .72;
            xTranslation =  gamepad1.left_stick_x * .72;
            rotation     = -gamepad1.right_stick_x * 0.9;
        }

        // Normal teleop drive control:
        // - left joystick is TRANSLATE fwd/back/left/right
        // - right joystick is ROTATE clockwise/counterclockwise
        // NOTE: assumes the right motors are defined FORWARD and the
        // left motors are defined REVERSE so positive power is FORWARD.
        frontRight = yTranslation - xTranslation + rotation;
        frontLeft  = yTranslation + xTranslation - rotation;
        rearRight  = yTranslation + xTranslation + rotation;
        rearLeft   = yTranslation - xTranslation - rotation;
        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                             Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        //auto break
        if ((gamepad1.left_stick_y < .15 && gamepad1.left_stick_y > -.15) && (gamepad1.left_stick_x < .15 && gamepad1.left_stick_x > -.15) && (gamepad1.right_stick_y < .15 && gamepad1.right_stick_y > -.15) && (gamepad1.right_stick_x < .15 && gamepad1.right_stick_x > -.15)) {
            frontLeft = 0;
            frontRight = 0;
            rearLeft = 0;
            rearRight = 0;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );


        /*
        //now for the legacy drive !
        //Finds the hypotenuse of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //Finds the robot's angle from the raw values of the joystick
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 || gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {
            //reversed bc idek the old code just works this way
            frontLeft = (-0.75*v2);
            frontRight = (-0.75*v1);
            rearLeft = (-0.75*v4);
            rearRight = (-0.75*v3);
        }

        else if(gamepad1.left_bumper) {
            frontLeft = (-0.5);
            rearLeft = (-0.5);
            frontRight = (0.5);
            rearRight = (0.5);
        }

        else if(gamepad1.right_bumper) {
            frontLeft = (0.5);
            rearLeft = (0.5);
            frontRight = (-0.5);
            rearRight = (-0.5);
        }
        else if(gamepad1.right_trigger > 0.15) {
            double power = 0.25*gamepad1.right_trigger;
            frontLeft = (power);
            rearLeft = (power);
            frontRight = (-power);
            rearRight = (-power);
        }
        else if(gamepad1.left_trigger > 0.15) {
            double power = 0.25*gamepad1.left_trigger;
            frontLeft = (-power);
            rearLeft = (-power);
            frontRight = (power);
            rearRight = (power);
        }
        else {
            frontLeft = (0);
            rearLeft = (0);
            frontRight = (0);
            rearRight = (0);
        }
        old code
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
        */

    } // processStandardDriveMode
    void encoderReset (){
        if (gamepad2_square_now && !gamepad2_square_last) { // reset tele-op positions
            robot.slideLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.slideRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.chainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.slideLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.slideRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.chainMotor.setTargetPosition(0);

            sleep(50);
            gamepad2.runRumbleEffect(shortRumble);
        }
    }
    void processLift(){

        if (gamepad2.triangle) { // precaution for incorrect tele-op starting position
            gamepad2.runRumbleEffect(shortRumble);
            if (gamepad2.right_bumper) {
                robot.slideLMotor.setPower(.5);
                robot.slideRMotor.setPower(.5);
            }
            else if (gamepad2.left_bumper) {
                robot.slideLMotor.setPower(-1);
                robot.slideRMotor.setPower(-1);
            }
        }
        else if( (gamepad2.left_bumper) && (robot.slideRMotorPos > 0) ) { //fast movement
                robot.slideLMotor.setPower(-1);
                robot.slideRMotor.setPower(-1);
        }
        else if (gamepad2.right_bumper) { // regular movement???
            if (robot.chainMotorPos > -940 && robot.slideRMotorPos > 1150) {
                robot.slideLMotor.setPower(0);
                robot.slideRMotor.setPower(0);
                gamepad2.runRumbleEffect(shortRumble);
            } else if (robot.chainMotorPos > -940 && robot.slideRMotorPos > 950) {
                robot.slideLMotor.setPower(.5);
                robot.slideRMotor.setPower(.5);
            } else if (robot.chainMotorPos < -940) {
                    robot.slideLMotor.setPower(1);
                    robot.slideRMotor.setPower(1);
            } else if (robot.chainMotorPos > -940 && robot.slideRMotorPos < 1150) {
                robot.slideLMotor.setPower(1);
                robot.slideRMotor.setPower(1);
            } else {
                robot.slideLMotor.setPower(0);
                robot.slideRMotor.setPower(0);
                gamepad2.runRumbleEffect(shortRumble);
            }
        }
        else {
            //precision movement
            if( (gamepad2.left_trigger >= 0.1) && (robot.slideRMotorPos >  0) ){
                if(gamepad2.left_trigger>=0.1 && gamepad2.left_trigger<= 0.3) {
                    robot.slideLMotor.setPower(-0.5*(gamepad2.left_trigger));
                    robot.slideRMotor.setPower(-0.5*(gamepad2.left_trigger));
                } else {
                    double motorPower = -1.3 * gamepad2.left_trigger;
                    if (motorPower < -1.0) motorPower = -1.0;
                    robot.slideLMotor.setPower(motorPower);
                    robot.slideRMotor.setPower(motorPower);
                }
            }
            else if ( (gamepad2.right_trigger >= 0.10) ){

                if (robot.chainMotorPos >= -940 && robot.slideRMotorPos > 1150) {
                    robot.slideLMotor.setPower(0);
                    robot.slideRMotor.setPower(0);
                    gamepad2.runRumbleEffect(shortRumble);
                } else if (robot.chainMotorPos > -940 && robot.slideRMotorPos > 850) {
                    robot.slideLMotor.setPower(.5*(gamepad2.right_trigger));
                    robot.slideRMotor.setPower(.5*(gamepad2.right_trigger));
                } else if (robot.chainMotorPos < -940) {
                    if(gamepad2.right_trigger>=0.1 && gamepad2.right_trigger<= 0.3) {
                        robot.slideLMotor.setPower(0.5*(gamepad2.right_trigger));
                        robot.slideRMotor.setPower(0.5*(gamepad2.right_trigger));
                    } else {
                        robot.slideLMotor.setPower(1 * (gamepad2.right_trigger));
                        robot.slideRMotor.setPower(1 * (gamepad2.right_trigger));
                    }
                } else if (robot.chainMotorPos > -940 && robot.slideRMotorPos < 1150) {
                    robot.slideLMotor.setPower(1*(gamepad2.right_trigger));
                    robot.slideRMotor.setPower(1*(gamepad2.right_trigger));
                } else {
                    robot.slideLMotor.setPower(0);
                    robot.slideRMotor.setPower(0);
                    gamepad2.runRumbleEffect(shortRumble);
                }
            }
            else {
                robot.slideLMotor.setPower(0);
                robot.slideRMotor.setPower(0);
            }
        }
        if(gamepad2.cross) {
            if (robot.slideRMotorPos < 1000){
                robot.slideLMotor.setPower(0.055);
                robot.slideRMotor.setPower(0.055);
            } else {
                robot.slideLMotor.setPower(0.07);
                robot.slideRMotor.setPower(0.07);
            }
        }
    } // processLift

    void chainMotorMovement(){
        //fix out of bounds
        if (gamepad2.triangle) {
            if (gamepad2.left_stick_y >= 0.15) {
                robot.chainMotorPos += gamepad2.left_stick_y * 100;
                robot.chainMotor.setTargetPosition(robot.chainMotorPos);
                robot.chainMotor.setPower(-1);
            } else if (gamepad2.left_stick_y <= -0.15) {
                robot.chainMotorPos += gamepad2.left_stick_y * 100;
                robot.chainMotor.setTargetPosition(robot.chainMotorPos);
                robot.chainMotor.setPower(-1);
            } else {
                robot.chainMotor.setPower(-1); //keeps chain just in position
            }
        }
        // limitation if not pressing triangle
        else if (robot.chainMotorPos <= -1800) {
            robot.chainMotorPos = -1700;
            gamepad2.runRumbleEffect(shortRumble);
        }
        else if (robot.chainMotorPos >= 10) {
            robot.chainMotorPos = -50;
            gamepad2.runRumbleEffect(shortRumble);

        }
        else if (robot.chainMotorPos <= -1800) {
            robot.chainMotorPos = -1700;
            gamepad2.runRumbleEffect(shortRumble);
        }
        else if (robot.chainMotorPos >= 10) {
            robot.chainMotorPos = -50;
            gamepad2.runRumbleEffect(shortRumble);

        }
        // high basket scoring preset
        if( gamepad2.dpad_up ){
            robot.chainMotor.setTargetPosition(-1656);
            robot.chainMotor.setPower(-1);
        }
        // wall intake scoring preset
        if( gamepad2.dpad_right ){
            if (robot.slideRMotorPos <= 1000) {
                robot.chainMotor.setTargetPosition(-273);
                robot.chainMotor.setPower(-1);
            }
        }
        // chamber scoring
        if (gamepad2.dpad_left) {
            robot.chainMotor.setTargetPosition(-1660);
            robot.chainMotor.setPower(-1);

        }


            // manual chain controls
            if (gamepad2.left_stick_y >= 0.15) {
                robot.chainMotorPos += gamepad2.left_stick_y * 70;
                robot.chainMotor.setTargetPosition(robot.chainMotorPos);
                robot.chainMotor.setPower(-1);
            } else if (gamepad2.left_stick_y <= -0.15) {
                robot.chainMotorPos += gamepad2.left_stick_y * 70;
                robot.chainMotor.setTargetPosition(robot.chainMotorPos);
                robot.chainMotor.setPower(-1);
            } else {
                robot.chainMotor.setPower(-1); //keeps chain just in position
            }

    }

    void diffyMovement(){
        //scoring and intake presets
        if( gamepad2.dpad_right ){ //preset for intake from wall
                robot.diffyLPos = 0.497; robot.diffyLServo.setPosition(robot.diffyLPos);
                robot.diffyRPos = 0.500; robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        else if (gamepad2.dpad_up) { //preset for scoring on basket
            robot.diffyLPos = 0.150;    robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.457;    robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        else if (gamepad2.dpad_left) { //preset for scoring on chambers
            robot.diffyLPos = 0.026; robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.570; robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        else if (gamepad2.dpad_down) { //horizontal scoring for basket
            robot.diffyLPos = 0.060; robot.diffyLServo.setPosition(robot.diffyLPos);
            robot.diffyRPos = 0.732; robot.diffyRServo.setPosition(robot.diffyRPos);
        }
        //manual differential rotation
            if (gamepad2.right_stick_y <= -.25) { //rotate yaw up (might be down)
                if (robot.diffyLPos > .015 && robot.diffyRPos < 0.985) {
                    robot.diffyLPos -= .015 * (-gamepad2.right_stick_y);
                    robot.diffyRPos += .015 * (-gamepad2.right_stick_y);
                } else {
                    gamepad2.runRumbleEffect(shortRumble);
                }
            } else if (gamepad2.right_stick_y >= .25) { //rotate yaw down (might be up)
                if (robot.diffyLPos < 0.985 && robot.diffyRPos > 0.015) {
                    robot.diffyLPos += -.015 * (-gamepad2.right_stick_y);
                    robot.diffyRPos -= -.015 * (-gamepad2.right_stick_y);
                } else {
                    gamepad2.runRumbleEffect(shortRumble);
                }
            }

            if (gamepad2.right_stick_x <= -.25) { //rotate roll right (might be left)
                if (robot.diffyLPos < 0.985 && robot.diffyRPos < 0.985) {
                    robot.diffyLPos += .015 * (gamepad2.right_stick_x);
                    robot.diffyRPos += .015 * (gamepad2.right_stick_x);
                } else {
                    gamepad2.runRumbleEffect(shortRumble);
                }
            } else if (gamepad2.right_stick_x >= .25) { //rotate roll left (might be right)
                if (robot.diffyLPos > 0.015 && robot.diffyRPos > 0.015) {
                    robot.diffyLPos -= -.015 * (gamepad2.right_stick_x);
                    robot.diffyRPos -= -.015 * (gamepad2.right_stick_x);
                } else {
                    gamepad2.runRumbleEffect(shortRumble);
                }
            }

        //rumble safeguard
        if (robot.diffyLPos >= 0.90) { //single rumble when left is getting to high, one rumble when its going too low
          gamepad2.runRumbleEffect(leftDoubleRumble);
        } else if (robot.diffyLPos <= .10) {
            gamepad2.runRumbleEffect(leftRumble);
        }

        if (robot.diffyRPos >= .90) { //single rumble when left is getting to high, one rumble when its going too low
            gamepad2.runRumbleEffect(rightDoubleRumble);
        } else if (robot.diffyRPos <= .10) {
            gamepad2.runRumbleEffect(rightRumble);
        }
        //set pos
        robot.diffyLServo.setPosition(robot.diffyLPos);
        robot.diffyRServo.setPosition(robot.diffyRPos);
    }

    void clawMovement() {
        //toggle for claw
        if (gamepad2_circle_now && !gamepad2_circle_last) { //controller input
            robot.clawOpen = !robot.clawOpen;
        }

        if (robot.clawOpen) //set value based on bool
        {
            robot.clawPos = 0.970;
            robot.clawServo.setPosition(robot.clawPos);
        }
        else
        {
            robot.clawPos = 0.855;
            robot.clawServo.setPosition(robot.clawPos);
        }

    }


} // Teleop
