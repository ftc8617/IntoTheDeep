package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import android.os.Environment;
import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public abstract class AutonomousBase extends LinearOpMode {

    // Define our hardware class here (so it's available in every Autonomous program we write
    HardwareMinibot robot = new HardwareMinibot();

    // gamepad controls for changing autonomous options
    boolean gamepad1_circle_last,   gamepad1_circle_now  =false;
    boolean gamepad1_cross_last,    gamepad1_cross_now   =false;
    boolean gamepad1_l_bumper_last, gamepad1_l_bumper_now=false;
    boolean gamepad1_r_bumper_last, gamepad1_r_bumper_now=false;
    boolean gamepad1_dpad_up_last, gamepad1_dpad_up_now = false;
    boolean gamepad1_dpad_down_last, gamepad1_dpad_down_now = false;
    boolean gamepad1_dpad_left_last, gamepad1_dpad_left_now = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    static final double MIN_DRIVE_POW = 0.05;    // Minimum speed to move the robot
    static final double MIN_SPIN_RATE = 0.05;    // Minimum power to turn the robot
    static final double MIN_DRIVE_MAGNITUDE = Math.sqrt(MIN_DRIVE_POW * MIN_DRIVE_POW + MIN_DRIVE_POW * MIN_DRIVE_POW);
    double robotGlobalXCoordinatePosition = 0.0;   // inches
    double robotGlobalYCoordinatePosition = 0.0;   // inches
    double robotOrientationRadians = 0.0;   // radians 0deg (straight forward)
    double pos_y = 0, pos_x = 0, pos_angle = 0.0;  // Allows us to specify movement INCREMENTALLY, not ABSOLUTE
    static final int DRIVE_TO = 1;   // ACCURACY: tighter tolerances, and slows then stops at final position
    static final int DRIVE_THRU = 2;   // SPEED: looser tolerances, and leave motors running (ready for next command)

    public void performEveryLoop () {
        // Get updated motor encoder values (in a single transaction)
        robot.readBulkData();
        // Command the goBilda Pinpoint to update the current position
        robot.odom.update();
        // Get updated odometry x/y/angle values from goBilda Pinpoint
        Pose2D pos = robot.odom.getPosition();  // x,y pos in inch; heading in radians
        robotGlobalXCoordinatePosition = pos.getX(DistanceUnit.INCH);
        robotGlobalYCoordinatePosition = pos.getY(DistanceUnit.INCH);
        robotOrientationRadians = pos.getHeading(AngleUnit.RADIANS);
    } // performEveryLoop'

    /**
     * Ensure angle is in the range of -PI to +PI (-180 to +180 deg)
     * @param angleRadians
     * @return
     */
    public double AngleWrapRadians( double angleRadians ){
        while( angleRadians < -Math.PI ) {
            angleRadians += 2.0*Math.PI;
        }
        while( angleRadians > Math.PI ){
            angleRadians -= 2.0*Math.PI;
        }
        return angleRadians;
    }

    /**
     * Ensure angle is in the range of -180 to +180 deg (-PI to +PI)
     * @param angleDegrees
     * @return
     */
    public double AngleWrapDegrees( double angleDegrees ){
        while( angleDegrees < -180 ) {
            angleDegrees += 360.0;
        }
        while( angleDegrees > 180 ){
            angleDegrees -= 360.0;
        }
        return angleDegrees;
    } // AngleWrapDegrees

    /*---------------------------------------------------------------------------------*/
    protected void captureGamepad1Buttons() {
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
    } // captureGamepad1Buttons

    /*--------------------------------------------------------------------------------------------*/
    /**
     * @param yTarget     - The Y field coordinate to go to.
     * @param xTarget     - The X field coordinate to go to.
     * @param angleTarget - The angle the robot should try to face when reaching destination in degrees.
     * @param speedMax    - Sets the speed when we are driving through the point.
     * @param driveType   - Slows the robot down to stop at destination coordinate.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected boolean driveToXY ( double yTarget, double xTarget, double angleTarget,
                                  double speedMax, int driveType){
        double errorMultiplier = 0.035;   // convert position-error to motor power TODO: FINE TUNE BASED ON AUTO EXPERIENCE
        double speedMin = MIN_DRIVE_MAGNITUDE;
        double allowedError = (driveType == DRIVE_THRU) ? 2.50 : 0.5;
        return (driveToXY(yTarget, xTarget, angleTarget, speedMin, speedMax, errorMultiplier,
                allowedError, driveType));
    } // driveToX

    public void driveToPosition ( double yTarget, double xTarget, double angleTarget,
                                  double speedMax, double turnMax, int driveType){
        // Update our current position
        performEveryLoop();
        // Loop until we get to destination.
        while (!driveToXY(yTarget, xTarget, angleTarget, speedMax, driveType)
                && opModeIsActive()) {
            // Keep updating our position
            performEveryLoop();
            telemetry.addData("Drive", "x=%.1f, y=%.1f, %.1f deg",
                    robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians));
            telemetry.update();
        } /* runOpMode() */
        if (driveType == DRIVE_TO) {
            rotateToAngle(angleTarget, turnMax);
        }
    } // driveToPosition

    /*--------------------------------------------------------------------------------------------*/
    /**
     * @param xTarget           - The X field coordinate to go to.
     * @param yTarget           - The Y field coordinate to go to.
     * @param angleTarget - The angle the robot should try to face when reaching destination in degrees.
     * @param speedMin    - The minimum speed that allows movement.
     * @param speedMax    - Sets the maximum speed to drive.
     * @param errorMultiplier - Sets the proportional speed to slow down.
     * @param errorAllowed - Sets the allowable error to claim target reached.
     * @param driveType - Allows waypoint to be a drive through where the robot won't slow down.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected boolean driveToXY ( double yTarget, double xTarget, double angleTarget,
                                  double speedMin,
                                  double speedMax, double errorMultiplier, double errorAllowed,
                                  int driveType){
        boolean reachedDestination = false;
        double xWorld = robotGlobalXCoordinatePosition;  // inches
        double yWorld = robotGlobalYCoordinatePosition;  // inches
        double xMovement = 0.0, yMovement = 0.0, turnMovement = 0.0;
        // Not sure why, but the x and y are backwards
        double deltaX = xTarget - xWorld;
        double deltaY = yTarget - yWorld;
        double driveAngle = Math.atan2(deltaY, deltaX);
        double deltaAngle = AngleWrapRadians(toRadians(angleTarget) - robotOrientationRadians);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double driveSpeed;
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
        // Have to convert from world angles to robot centric angles.
        double robotDriveAngle = driveAngle - robotOrientationRadians;

        // This will allow us to do multi-point routes without huge slowdowns.
        // Such use cases will be changing angles, or triggering activities at
        // certain points.
        if (!(driveType == DRIVE_THRU)) {
            driveSpeed = magnitude * errorMultiplier;
        } else {
            driveSpeed = speedMax;
        }

        if (driveSpeed < speedMin) {
            driveSpeed = speedMin;
        } else if (driveSpeed > speedMax) {
            driveSpeed = speedMax;
        }

        // Check if we passed through our point
        if (magnitude <= errorAllowed) {
            reachedDestination = true;
            if (!(driveType == DRIVE_THRU)) {
                robot.stopMotion();
            } else {
                // This can happen if the robot is already at error distance for drive through
                xMovement = driveSpeed * Math.cos(robotDriveAngle);
                yMovement = driveSpeed * Math.sin(robotDriveAngle);
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        } else {
            xMovement = driveSpeed * Math.cos(robotDriveAngle);
            yMovement = driveSpeed * Math.sin(robotDriveAngle);
            turnMovement = turnSpeed;
            ApplyMovement(yMovement, xMovement, turnMovement);
        } // driveToXY

        boolean ODOMETRY_DEBUG = false;
        if (ODOMETRY_DEBUG) {
            sleep(250);       // allow 0.25 seconds of progress (from prior loop)...
            robot.driveTrainMotorsZero(); // then stop and observe
            telemetry.addData("World X (inches)", "%.2f in", xWorld);
            telemetry.addData("World Y (inches)", "%.2f in", yWorld);
            telemetry.addData("Orientation (deg)", "%.2f deg", Math.toDegrees(robotOrientationRadians));
            telemetry.addData("distanceToPoint", "%.2f in", magnitude);
            telemetry.addData("angleToPoint", "%.4f deg", Math.toDegrees(driveAngle));
            telemetry.addData("deltaAngleToPoint", "%.4f deg", Math.toDegrees(deltaAngle));
            telemetry.addData("relative_x_to_point", "%.2f in", deltaX);
            telemetry.addData("relative_y_to_point", "%.2f in", deltaY);
            //telemetry.addData("robot_radian_err", "%.4f deg", Math.toDegrees(robot_radian_err));
            telemetry.addData("movement_x_power", "%.2f", xMovement);
            telemetry.addData("movement_y_power", "%.2f", yMovement);
            telemetry.addData("rotation_power", "%.2f", turnMovement);
            telemetry.update();
            sleep(5000);  // so we can read the output above
        } // ODOMETRY_DEBUG

        return reachedDestination;
    } // driveToXY

    /*--------------------------------------------------------------------------------------------*/
    /**
     * @param angleTarget  - The angle the robot should try to face when reaching destination.
     * @param turnMax - Highest motor power to use for turn speed
     */
    public void rotateToAngle ( double angleTarget, double turnMax){
        // Update our current position
        performEveryLoop();
        // Start the rotation, and reset our reference angle
        rotateToAngle(angleTarget, true);
        // Update our current position
        performEveryLoop();
        // Loop until we've rotated to the desired angle
        while (!rotateToAngle(angleTarget, false) && opModeIsActive()) {
            performEveryLoop();
        }
    } // rotateToAngle

    /*--------------------------------------------------------------------------------------------*/
    /**
     * @param angleTarget  - The angle the robot should try to face when reaching destination.
     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected double lastDriveAngle;
    protected boolean rotateToAngle ( double angleTarget, boolean resetDriveAngle){
        boolean reachedDestination = false;
        double xMovement, yMovement, turnMovement;
        double errorMultiplier = 0.016;
        double turnMin = MIN_SPIN_RATE;
        double deltaAngle = AngleWrapRadians(toRadians(angleTarget) - robotOrientationRadians);
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;

        // This should be set on the first call to start us on a new path.
        if (resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

        // We are done if we are within 2.0 degrees
        if (Math.abs(Math.toDegrees(deltaAngle)) < 2.0) {
            // We have reached our destination if the angle is close enough
            robot.stopMotion();
            reachedDestination = true;
            // We are done when we flip signs.
        } else if (lastDriveAngle < 0) {
            // We have reached our destination if the delta angle sign flips from last reading
            if (deltaAngle >= 0) {
                robot.stopMotion();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                xMovement = 0;
                yMovement = 0;
                if (turnSpeed > -turnMin) {
                    turnSpeed = -turnMin;
                }
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        } else {
            // We have reached our destination if the delta angle sign flips
            if (deltaAngle <= 0) {
                robot.stopMotion();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                xMovement = 0;
                yMovement = 0;
                if (turnSpeed < turnMin) {
                    turnSpeed = turnMin;
                }
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        }
        lastDriveAngle = deltaAngle;

        return reachedDestination;

    } // rotateToAngle
    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts xMovement, movement_x, movement_turn into motor powers */
    public void ApplyMovement ( double yMovement, double xMovement, double turnMovement){
        long currTime = SystemClock.uptimeMillis();
        // wait at least 16 msec before expecting a position update
        if (currTime - lastUpdateTime < 16) {
            return;
        }
        lastUpdateTime = currTime;

        double frontRight = yMovement - xMovement + turnMovement;
        double frontLeft = yMovement + xMovement - turnMovement;
        double backRight = yMovement + xMovement + turnMovement;
        double backLeft = yMovement - xMovement - turnMovement;

        //find the maximum of the powers
        double maxRawPower = Math.abs(frontLeft);
        if (Math.abs(backLeft) > maxRawPower) {
            maxRawPower = Math.abs(backLeft);
        }
        if (Math.abs(backRight) > maxRawPower) {
            maxRawPower = Math.abs(backRight);
        }
        if (Math.abs(frontRight) > maxRawPower) {
            maxRawPower = Math.abs(frontRight);
        }

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if (maxRawPower > 1.0) {
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0 / maxRawPower;
        }
        frontLeft *= scaleDownAmount;
        backLeft *= scaleDownAmount;
        backRight *= scaleDownAmount;
        frontRight *= scaleDownAmount;

        robot.driveTrainMotors(frontLeft, frontRight, backLeft, backRight);
    } // ApplyMovement

} // AutonomousBase
