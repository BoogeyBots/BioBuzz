package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Configurable
public class Test_motor extends LinearOpMode {

    DcMotorEx motor, motor2;
    VoltageSensor voltageSensor; // Senzorul pentru voltajul sistemului

    public static double pow1, pow2;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor_transfer");

        // Preluăm senzorul de voltaj de pe Control Hub
        voltageSensor = hardwareMap.voltageSensor.get("Control Hub");

        waitForStart();

        while (opModeIsActive()) {
            // Setăm puterea motoarelor
            motor.setPower(pow1);
            motor2.setPower(pow2);

            // Citim curentul consumat de fiecare motor în Amperi
            double curentIntake = motor.getCurrent(CurrentUnit.AMPS);
            double curentTransfer = motor2.getCurrent(CurrentUnit.AMPS);

            // Citim voltajul bateriei
            double voltajBaterie = voltageSensor.getVoltage();

            // Afișăm valorile în Telemetry pe Driver Station
            telemetry.addData("Intake - Putere", pow1);
            telemetry.addData("Intake - Curent", "%.2f A", curentIntake);

            telemetry.addData("Transfer - Putere", pow2);
            telemetry.addData("Transfer - Curent", "%.2f A", curentTransfer);

            telemetry.addData("Voltaj Baterie", "%.2f V", voltajBaterie);
            telemetry.update();
        }
    }
}