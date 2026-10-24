package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
@TeleOp
public class outtake_test extends LinearOpMode {
    DcMotorEx motordr, motorst, motor_intake, motor_transfer;
    public static double targe, p = 0.003, ks = 0.1, kv = 0.000429266, ka = 0.005, velocity, manual_pow;
    public static boolean manual = false;
    double voltage;
    PIDFController controller = new PIDFController(p, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    @Override
    public void runOpMode() throws InterruptedException {
        motordr = hardwareMap.get(DcMotorEx.class, "dreapta");
        motorst = hardwareMap.get(DcMotorEx.class, "stanga");

        motordr.setDirection(DcMotorSimple.Direction.REVERSE);

        TelemetryManager panelsTelemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            velocity = motordr.getVelocity();

            update();
            panelsTelemetry.addData("Velocity", velocity);
            panelsTelemetry.addData("Target", targe);
            panelsTelemetry.addData("Voltage", voltage);
            panelsTelemetry.addData("AMPS st", motorst.getCurrent(CurrentUnit.AMPS));
            panelsTelemetry.addData("AMPS dr", motordr.getCurrent(CurrentUnit.AMPS));
            panelsTelemetry.update(telemetry);

            if(manual == true) {
                motordr.setPower(manual_pow);
                motorst.setPower(manual_pow);
            }
        }

    }

    public void update() {
        controller.setPIDF(p, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        double PID_output = controller.calculate(velocity, targe);
        double ff_output = feedforward.calculate(targe);

        double output = PID_output + ff_output;

        if(targe != 0) {
            motordr.setPower(output * (9.3 / voltage));
            motorst.setPower(output * (9.3 / voltage));
        }

        else {
            motordr.setPower(0);
            motorst.setPower(0);
        }
    }
}




