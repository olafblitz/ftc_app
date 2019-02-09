package org.firstinspires.ftc.teamcode.OutOfOrder14235.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OutOfOrder14235.Autonomous.HardwareRobot;

import java.util.Locale;

@TeleOp

public class NormalTeleOpASNOGYRO extends LinearOpMode {
    HardwareRobot robot;
    ModernRoboticsI2cGyro gyro;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private Gyroscope imu;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor centerWheel;
    private DcMotor linearActuator;
    private Servo markerDropper;
    private DcMotor linExt;
    DistanceSensor distanceBottom;
    ColorSensor colorSideLeft;
    DistanceSensor distanceSideLeft;
    ColorSensor colorSideRight;
    DistanceSensor distanceSideRight;
    DigitalChannel touchLeft;
    DigitalChannel  touchRight;
    ColorSensor colorMarker;
    DistanceSensor distanceMarker;

    private DcMotor flipper;

    @Override
    public void runOpMode() {
        robot = new HardwareRobot();
        robot.init(hardwareMap);
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double trigger;

        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftWheel = hardwareMap.get(DcMotor.class, "left_drive");
        rightWheel = hardwareMap.get(DcMotor.class, "right_drive");
        centerWheel = hardwareMap.get(DcMotor.class, "pulleyMotor");
        markerDropper = hardwareMap.get(Servo.class, "markerDropper");
        linearActuator = hardwareMap.get(DcMotor.class, "wormGear");
        linExt = hardwareMap.get(DcMotor.class, "linExt");
        flipper = hardwareMap.get(DcMotor.class, "flipper");
        distanceBottom = hardwareMap.get(DistanceSensor.class, "distanceBottom");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distanceBottom;
        colorSideLeft = hardwareMap.get(ColorSensor.class, "colorDistanceLeft");
        colorSideRight = hardwareMap.get(ColorSensor.class, "colorDistanceRight");

        distanceSideLeft = hardwareMap.get(DistanceSensor.class, "colorDistanceLeft");
        distanceSideRight = hardwareMap.get(DistanceSensor.class, "colorDistanceRight");
        touchLeft = hardwareMap.get(DigitalChannel.class, "touchSensorLeft");
        touchLeft.setMode(DigitalChannel.Mode.INPUT);
        touchRight = hardwareMap.get(DigitalChannel.class, "touchSensorRight");
        touchRight.setMode(DigitalChannel.Mode.INPUT);
        colorMarker = hardwareMap.get(ColorSensor.class, "colorMarker");
        distanceMarker = hardwareMap.get(DistanceSensor.class, "colorMarker");
        gyro =  hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        float hsvValuesLeft[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float valuesLeft[] = hsvValuesLeft;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTORLEFT = 255;
        float hsvValuesRight[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float valuesRight[] = hsvValuesRight;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTORRIGHT = 255;
        telemetry.addData("Status", "Ready to start!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Color.RGBToHSV((int) (colorSideLeft.red() * SCALE_FACTORLEFT),
                    (int) (colorSideLeft.green() * SCALE_FACTORLEFT),
                    (int) (colorSideLeft.blue() * SCALE_FACTORLEFT),
                    hsvValuesLeft);

            // send the info back to driver station using telemetry function.

            telemetry.addData("LEFT Red  ", colorSideLeft.red());
            telemetry.addData("LEFT Blue ", colorSideLeft.blue());

            Color.RGBToHSV((int) (colorSideRight.red() * SCALE_FACTORRIGHT),
                    (int) (colorSideRight.green() * SCALE_FACTORRIGHT),
                    (int) (colorSideRight.blue() * SCALE_FACTORRIGHT),
                    hsvValuesRight);

            // send the info back to driver station using telemetry function.

            telemetry.addData("RIGHT Red  ", colorSideRight.red());
            telemetry.addData("RIGHT Blue ", colorSideRight.blue());


            telemetry.addData("range", String.format("%.01f centimeters bottom distance sensor", distanceBottom.getDistance(DistanceUnit.CM)));
            telemetry.addData("linear actuator encoder position: ", linearActuator.getCurrentPosition());
            telemetry.update();
            // trigger = gamepad1.right_trigger;
            //drive = -gamepad1.left_stick_y;
            //  turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            //left  = drive + turn;
            //right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            //  max = Math.max(Math.abs(left), Math.abs(right));
            //if (max > 1.0)
            //{
            //    left /= max;
            //    right /= max;
            // }

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            left  = drive + turn;
            right = drive - turn;
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftWheel.setPower(left);
            rightWheel.setPower(right);
            if(gamepad2.right_trigger>0){
                flipper.setPower(.4);
            }
            else if(gamepad2.left_trigger>0){
                flipper.setPower(-.4);

            }
            else{
                flipper.setPower(0);
            }

            if(gamepad1.y){
                linearActuator.setPower(-1);
            }
            else if(gamepad1.a){
                linearActuator.setPower(1);
            }
            else{
                linearActuator.setPower(0);
            }
            if(gamepad1.x){
                centerWheel.setPower(.7);
            }
            else if(gamepad1.b){
                centerWheel.setPower(-.7);
            }
            else{
                centerWheel.setPower(0);
            }
            if (gamepad1.dpad_left){
                markerDropper.setPosition(.3);
            }
            else if (gamepad1.dpad_right){
                markerDropper.setPosition(0.75);

            }
            if(gamepad2.y){
                linExt.setPower(-1);
            }
            else if(gamepad2.a){
                linExt.setPower(1);
            }
            else{
                linExt.setPower(0);
            }
            if(gamepad1.dpad_down){
                robot.sideArm.setPower(-1);
            }
            else if(gamepad1.dpad_up){
                robot.sideArm.setPower(1);
            }
            else{
                robot.sideArm.setPower(0);
            }
            if(gamepad2.b){
                //TODO
            }


        }
        telemetry.addData("Status", "Running");

        telemetry.update();

    }

    public void waiting(long millis){
        long t = System.currentTimeMillis();
        while(opModeIsActive()&&System.currentTimeMillis()-t<millis){

        }
    }

    double DRIVE_POWER = 1.0;
    public void TurnLeft(double lPower, double rPower){
        robot.leftWheel.setPower(lPower);
        robot.rightWheel.setPower(rPower);

    }
    public void StopMoving (){
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        robot.centerWheel.setPower(0);
    }
    public void TurnRight(double lPower, double rPower){
        robot.leftWheel.setPower(lPower);
        robot.rightWheel.setPower(rPower);

    }

    public void DriveForward(double power){
        robot.leftWheel.setPower(power);
        robot.rightWheel.setPower(power);

    }
    public void DriveBackward(double power){
        robot.leftWheel.setPower(-power);
        robot.rightWheel.setPower(-power);

    }
    public void ShiftLeft(double power){
        robot.centerWheel.setPower(power);
    }
    public void ShiftRight(double power){
        robot.centerWheel.setPower(-power);
    }
    public void StopDriving(){
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        robot.centerWheel.setPower(0);

    }


    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftWheel.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftWheel.setTargetPosition(newLeftTarget);
            robot.rightWheel.setTargetPosition(newRightTarget);

            robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftWheel.setPower(speed);
            robot.rightWheel.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftWheel.isBusy() && robot.rightWheel.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftWheel.setPower(leftSpeed);
                robot.rightWheel.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftWheel.getCurrentPosition(),
                        robot.rightWheel.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftWheel.setPower(0);
            robot.rightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     **/
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftWheel.setPower(leftSpeed);
        robot.rightWheel.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



}

