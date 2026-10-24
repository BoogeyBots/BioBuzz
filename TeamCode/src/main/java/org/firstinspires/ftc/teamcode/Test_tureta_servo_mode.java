package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class Test_tureta_servo_mode extends LinearOpMode {
    Servo s1, s2;

    // Unghiul dorit la iesirea turetei (in grade)
    public static double outputAngle = 180;

    // Ratia exacta calculata din angrenaje ((60/20) * (25/65) = 1.153846)
    private static final double GEAR_RATIO = (60.0 / 20.0) * (25.0 / 65.0);

    @Override
    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.get(Servo.class, "servo_right");
        s2 = hardwareMap.get(Servo.class, "servo_left");

        // Daca servourile sunt montate opus si se blocheaza, decomenteaza linia de mai jos:
        // s2.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // 1. Calculam unghiul pe care trebuie sa il faca servo-ul
            double servoAngle = outputAngle / GEAR_RATIO;

            // 2. Transformam unghiul servoului (0 - 300 grade) in valoare de pozitie
            double servoPosition = servoAngle / 300.0;

            // Trimitem comanda direct, fara nicio limitare
            s1.setPosition(servoPosition);
            s2.setPosition(servoPosition);

            // Telemetrie
            telemetry.addData("Tureta Angle (°)", outputAngle);
            telemetry.addData("Servo Angle (°)", servoAngle);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}
