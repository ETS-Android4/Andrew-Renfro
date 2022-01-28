package org.firstinspires.ftc.teamcode;
// adb connect 192.168.43.1:5555

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class TestRobot {

    static final double COUNTS_PER_MOTOR_REV = 5281.1 ;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = (1/13.7) * 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public Telemetry tel = null;
    HardwareMap hwm;

    public int arm1Target = 0;
    //public int arm2Target = 0;

    BNO055IMU imu;
    // Names of the motors and servos in the control hub
    public DcMotor Frontleft = null;
    public DcMotor Frontright = null;
    public DcMotor Backleft = null;
    public DcMotor Backright = null;
    public DcMotor arm1 = null;
    //public DcMotor arm2 = null;
    DcMotor Bob = null;
    Servo servo1 = null;

    private boolean initialized = false;

    public TestRobot(HardwareMap _hwm) {
        this.hwm = _hwm;
    }
    // Intialize robot and code under this will start on intializization.
    public void init()   {
        if (initialized) return;
        Frontleft = hwm.get(DcMotor.class, "Frontleft");
        Frontright = hwm.get(DcMotor.class, "Frontright");
        Backleft = hwm.get(DcMotor.class, "Backleft");
        Backright = hwm.get(DcMotor.class, "Backright");
        Bob = hwm.get(DcMotor.class, "Bob");
        arm1 = hwm.get(DcMotor.class, "arm1");

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setPower(1);
        arm1Target = arm1.getCurrentPosition();
        arm1.setTargetPosition(arm1Target);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //arm2 = hwm.get(DcMotor.class, "arm2");
        //arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm2.setPower(1);
        //arm2Target = arm1.getCurrentPosition();
        //arm2.setTargetPosition(arm2Target);
        //arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Frontleft.setDirection(DcMotor.Direction.REVERSE);
        Backleft.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwm.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initialized = true;
    }

    public void setMode(DcMotor.RunMode mode) {
        Frontleft.setMode(mode);
        Frontright.setMode(mode);
        Backleft.setMode(mode);
        Backright.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        Frontleft.setZeroPowerBehavior(mode);
        Frontright.setZeroPowerBehavior(mode);
        Backleft.setZeroPowerBehavior(mode);
        Backright.setZeroPowerBehavior(mode);
    }

    public void moveArm1(int delta) {
        arm1Target += delta;
        arm1.setTargetPosition(arm1Target);
    }

   /* public void moveArm2(int delta) {
        arm2Target += delta;
        arm2.setTargetPosition(arm2Target);
    }
    */

    // Code for Drive
    public void setDrivePower(float power) {
        Frontleft.setPower(power);
        Frontright.setPower(power);
        Backleft.setPower(power);
        Backright.setPower(power);
    }
    // Code for Turn
    public void setTurnPower(float power) {
        Frontleft.setPower(power);
        Frontright.setPower(-power);
        Backleft.setPower(power);
        Backright.setPower(-power);
    }
    // Code for Strafe
    public void setStrafePower(float power) {
        Frontleft.setPower(-power);
        Frontright.setPower(power);
        Backleft.setPower(power);
        Backright.setPower(-power);
    }

    public void brake() {
        setDrivePower(0);
    }

    public void encoderDrive(float distance, float power, int timeout) {
        distance *= -1;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Frontleft.setTargetPosition(
                Frontleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        Frontright.setTargetPosition(
                Frontright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        Backleft.setTargetPosition(
                Backleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        Backright.setTargetPosition(
                Backright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        setDrivePower(1);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (timer.time() <= timeout &&
                (Frontleft.isBusy() || Frontright.isBusy() ||
                        Backleft.isBusy() || Backright.isBusy())) {
            if (tel != null) {
                tel.addData("Current: ", Frontleft.getCurrentPosition());
                tel.addData("Target: ", Frontleft.getTargetPosition());
                tel.update();
            }
        }
        brake();
    }

    public void encoderStrafe(double distance, float power, int timeout) {
        distance *= -1;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);Frontleft.setTargetPosition(
                Frontleft.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH)
        );
        Frontright.setTargetPosition(
                Frontright.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        Backleft.setTargetPosition(
                Backleft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH)
        );
        Backright.setTargetPosition(
                Backright.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH)
        );

        setDrivePower(1);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (timer.time() <= timeout &&
                (Frontleft.isBusy() || Frontright.isBusy() ||
                        Backleft.isBusy() || Backright.isBusy())) {
            if (tel != null) {
                tel.addData("Current: ", Frontleft.getCurrentPosition());
                tel.addData("Target: ", Frontleft.getTargetPosition());
                tel.update();
            }
        }
        brake();
    }

    public void imuTurn(double target, int timeout) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double error = getError(target);
        double TOLERANCE = 0.5;
        if (error > 0) setTurnPower(-0.5f);
        else setTurnPower(0.5f);
        while (180 - Math.abs(error) > TOLERANCE) {

            error = getError(target);
            tel.addData("error", 180 - Math.abs(error));
            tel.update();
        }
        brake();
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getError(double target) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = target - getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
}

