package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

/*
 * Hardware class for goBilda robot (12"x15" chassis with 96mm/3.8" goBilda mecanum wheels)
 */
public class HardwareZawg
{
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;
    public double controlHubV   = 0.0; // Voltage supply of the control hub
    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected BNO055IMU imu    = null;
    public double headingAngle = 0.0;
    public double tiltAngle    = 0.0;
    protected GoBildaPinpointDriver odom = null;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor     = null;
    public int          frontLeftMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          frontLeftMotorPos  = 0;       // current encoder count
    public double       frontLeftMotorVel  = 0.0;     // encoder counts per second
    public double       frontLeftMotorAmps = 0.0;     // current power draw (Amps)

    protected DcMotorEx frontRightMotor    = null;
    public int          frontRightMotorTgt = 0;       // RUN_TO_POSITION target encoder count
    public int          frontRightMotorPos = 0;       // current encoder count
    public double       frontRightMotorVel = 0.0;     // encoder counts per second
    public double       frontRightMotorAmps= 0.0;     // current power draw (Amps)

    protected DcMotorEx rearLeftMotor      = null;
    public int          rearLeftMotorTgt   = 0;       // RUN_TO_POSITION target encoder count
    public int          rearLeftMotorPos   = 0;       // current encoder count
    public double       rearLeftMotorVel   = 0.0;     // encoder counts per second
    public double       rearLeftMotorAmps  = 0.0;     // current power draw (Amps)

    protected DcMotorEx rearRightMotor     = null;
    public int          rearRightMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          rearRightMotorPos  = 0;       // current encoder count
    public double       rearRightMotorVel  = 0.0;     // encoder counts per second
    public double       rearRightMotorAmps = 0.0;     // current power draw (Amps)

    public final static double MIN_DRIVE_POW      = 0.03;    // Minimum speed to move the robot
    public final static double MIN_TURN_POW       = 0.03;    // Minimum speed to turn the robot
    public final static double MIN_STRAFE_POW     = 0.04;    // Minimum speed to strafe the robot
    protected double COUNTS_PER_MOTOR_REV  = 28.0;    // goBilda Yellow Jacket Planetary Gear Motor Encoders
//  protected double DRIVE_GEAR_REDUCTION  = 26.851;  // goBilda 26.9:1 (223rpm) gear ratio with 1:1 HDT5 pully/belt
    protected double DRIVE_GEAR_REDUCTION  = 19.203;  // goBilda 19.2:1 (312rpm) gear ratio with 1:1 HDT5 pully/belt
    protected double MECANUM_SLIPPAGE      = 1.01;    // one wheel revolution doesn't achieve 6" x 3.1415 of travel.
    protected double WHEEL_DIAMETER_INCHES = 3.77953; // (96mm) -- for computing circumference
    protected double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * MECANUM_SLIPPAGE) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // The math above assumes motor encoders.  For REV odometry pods, the counts per inch is different
    protected double COUNTS_PER_INCH2      = 1738.4;  // 8192 counts-per-rev / (1.5" omni wheel * PI)

    protected DcMotorEx slideLMotor     = null;
    public int          slideLMotorPos  = 0;       // EH1
    public double       slideLMotorVel  = 0.0;     // EH1

    protected DcMotorEx slideRMotor     = null;
    public int          slideRMotorPos  = 0;       // EH2
    public double       slideRMotorVel  = 0.0;     // EH2

    public int          slideMotorTgt  = 0;       // EH1
    public boolean      slideMotorAuto = false;
    public boolean      slideMotorBusy = false;
    public int positionLError;
    public int positionRError;
    protected DcMotorEx chainMotor     = null;
    public int          chainMotorTgt  = 0;       // EH0
    public int          chainMotorPos  = 0;       // EH0
    public double       chainMotorVel  = 0.0;     // EH0

    public Servo clawServo   = null;
    public Servo diffyLServo = null;
    public Servo diffyRServo = null;

    public double  clawPos   = 0.565;
    public double  diffyLPos = 0.846;
    public double  diffyRPos = 0.202;
    public boolean clawOpen = false; 

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareZawg(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean isAutonomous ) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Configure REV control/expansion hubs for bulk reads (faster!)
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            if(module.isParent()) {
                controlHub = module;
            } else {
                expansionHub = module;
            }
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
        // Locate the odometry controller in our hardware settings
        odom = hwMap.get(GoBildaPinpointDriver.class,"odom"); //Control Hub I2C port 3
        odom.setOffsets(111.23, 36.73);   // odometry pod x,y locations [mm] relative to center of robot
        odom.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD ); // 4bar pods
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                  GoBildaPinpointDriver.EncoderDirection.FORWARD);
        if( isAutonomous ) {
            odom.resetPosAndIMU();
        }

        // Define and Initialize drivetrain motors
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"leftFront");  // Expansion Hub port 1 (REVERSE)
        frontRightMotor = hwMap.get(DcMotorEx.class,"rightFront"); // Expansion Hub port 2 (forward)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"leftBack");   // Expansion Hub port 0 (REVERSE)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"rightBack");  // Expansion Hub port 3 (forward)

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all drivetrain motors to zero power
        driveTrainMotorsZero();

        // Set all drivetrain motors to run with encoders.
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Define and initialize slide motors
        slideLMotor  = hwMap.get(DcMotorEx.class,"leftSlideMotor");
        slideRMotor  = hwMap.get(DcMotorEx.class,"rightSlideMotor");

        slideLMotor.setDirection(DcMotor.Direction.REVERSE);
        slideRMotor.setDirection(DcMotor.Direction.FORWARD);

        if(isAutonomous) {
            slideLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        slideLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideLMotor.setPower(0);
        slideRMotor.setPower(0);

        slideLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize the chain motor
        chainMotor  = hwMap.get(DcMotorEx.class,"chainMotor");
        chainMotor.setDirection(DcMotor.Direction.FORWARD);
        if(isAutonomous) {
            chainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chainMotor.setTargetPosition(0);
        } else {
            chainMotor.setTargetPosition(chainMotorPos);
        }
        chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chainMotor.setPower(0);

        // Initialize robot hardware (autonomous=true initializes servos)
        clawServo   = hwMap.get(Servo.class,"claw");            // servo port 0 (Expansion Hub)
        diffyLServo = hwMap.get(Servo.class,"diffyL");          // servo port 1 (Expansion Hub)
        diffyRServo = hwMap.get(Servo.class,"diffyR");         // servo port 2 (Control Hub)

        // Preload each variable with the initialization position
        if( isAutonomous ) {
          diffyLPos = 0.846;    diffyLServo.setPosition(diffyLPos);
          diffyRPos = 0.202;    diffyRServo.setPosition(diffyRPos);
          clawOpen = false;
        }

        // Initialize REV Control Hub IMU
//      initIMU();

    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    public void resetEncoders() {
        odom.resetPosAndIMU();
        clawPos   = 0.29;     clawServo.setPosition(clawPos);
       diffyLPos = 0.846;    diffyLServo.setPosition(diffyLPos);
       diffyRPos = 0.202;    diffyRServo.setPosition(diffyRPos);
       clawOpen = false;
       chainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       slideLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       slideRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    } // resetEncoders

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Define and initialize REV Expansion Hub IMU
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json"; // located in FIRST/settings folder
        imu_params.loggingEnabled = false;
        imu_params.loggingTag = "IMU";
        imu_params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize( imu_params );
    } // initIMU()

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMU()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
        headingAngle = angles.firstAngle;
        tiltAngle = angles.secondAngle;
        return -headingAngle;  // degrees (+90 is CW; -90 is CCW)
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
        expansionHub.clearBulkCache();
        controlHub.clearBulkCache();
        // Get a fresh set of values for this cycle
        //   getCurrentPosition() / getTargetPosition() / getTargetPositionTolerance()
        //   getPower() / getVelocity() / getCurrent()
        //===== CONTROL HUB VALUES =====
        frontLeftMotorPos  = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel  = frontLeftMotor.getVelocity();
        frontRightMotorPos = frontRightMotor.getCurrentPosition();
        frontRightMotorVel = frontRightMotor.getVelocity();
        rearRightMotorPos  = rearRightMotor.getCurrentPosition();
        rearRightMotorVel  = rearRightMotor.getVelocity();
        rearLeftMotorPos   = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel   = rearLeftMotor.getVelocity();
        slideLMotorPos     = slideLMotor.getCurrentPosition();
        slideRMotorPos     = slideRMotor.getCurrentPosition();
        chainMotorPos      = chainMotor.getCurrentPosition();
    } // readBulkData

    /*--------------------------------------------------------------------------------------------*/
    // This is a slow operation (involves an I2C reading) so only do it as needed
    public double readBatteryControlHub() {
        // Update local variable and then return that value
        controlHubV = controlHub.getInputVoltage( VoltageUnit.MILLIVOLTS );
        return controlHubV;
    } // readBatteryControlHub


    /*--------------------------------------------------------------------------------------------*/
    public void autoSlidePositionStart( int targetPos, double movePower )
    {
        // Save the target position
        slideMotorTgt = targetPos;
        slideMotorAuto = true;
        slideMotorBusy = true;
    } // autoSlidePositionStart
    public void autoSlidePositionAbort()
    {
        slideMotorAuto = false;
        slideMotorBusy = false;
    } // autoSlidePositionAbort
    public void autoSlidePositionUpdate()
    {
        positionLError = slideMotorTgt - slideLMotorPos;
        positionRError = slideMotorTgt - slideRMotorPos;
    } // autoSlidePositionUpdate

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotors( double frontLeft, double frontRight, double rearLeft, double rearRight )
    {
        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        rearLeftMotor.setPower( rearLeft );
        rearRightMotor.setPower( rearRight );
    } // driveTrainMotors

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to drive straight FORWARD (Ex: +0.10) or REVERSE (Ex: -0.10)        */
    public void driveTrainFwdRev( double motorPower )
    {
        frontLeftMotor.setPower(  motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(   motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainFwdRev

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to strafe RIGHT (Ex: +0.10) or LEFT (Ex: -0.10)                     */
    public void driveTrainRightLeft( double motorPower )
    {
        frontLeftMotor.setPower(   motorPower );
        frontRightMotor.setPower( -motorPower );
        rearLeftMotor.setPower(   -motorPower );
        rearRightMotor.setPower(   motorPower );
    } // driveTrainRightLeft

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to turn clockwise (Ex: +0.10) or counterclockwise (Ex: -0.10)       */
    public void driveTrainTurn( double motorPower )
    {
        frontLeftMotor.setPower( -motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(  -motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainTurn

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotorsZero()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        rearLeftMotor.setPower( 0.0 );
        rearRightMotor.setPower( 0.0 );
    } // driveTrainMotorsZero

    /*--------------------------------------------------------------------------------------------*/
    public void stopMotion() {
        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    /*--------------------------------------------------------------------------------------------*/

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    } /* waitForTick() */

} /* HardwareMinibot */
