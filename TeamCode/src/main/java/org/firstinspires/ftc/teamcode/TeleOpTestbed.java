/* FTC Team 8617 - Version 1.0 (10/01/2022)
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TeleOp (with test modes).
 */
@TeleOp(name="TeleOpTestbed", group="8617")
public class TeleOpTestbed extends LinearOpMode {
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
    HardwareTestbed robot = new HardwareTestbed();
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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "austen james richardson");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Normally autonomous resets encoders.  Do we need to for teleop??
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

} // Teleop
