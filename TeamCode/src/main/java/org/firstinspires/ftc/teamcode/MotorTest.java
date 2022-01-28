package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Motor Test", group="Linear Opmode")
public class MotorTest extends OpMode {
    MechanumHardware hwm;
    @Override
    public void init() {
        hwm = new MechanumHardware(hardwareMap);
        hwm.init();
    }

    @Override
    public void loop() {
        hwm.frontLeftDrive.setPower(gamepad1.a ? 1.0 : 0.0);
        hwm.frontRightDrive.setPower(gamepad1.b ? 1.0 : 0.0);
        hwm.backLeftDrive.setPower(gamepad1.x ? 1.0 : 0.0);
        hwm.backRightDrive.setPower(gamepad1.y ? 1.0 : 0.0);
    }
}
