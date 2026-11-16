package org.firstinspires.ftc.teamcode.Test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
@TeleOp(name = "test intake")
public class test_intake extends OpMode {

    private DcMotorEx motorIntake, motorTransfer;
    private Servo servoFaras, servoblocaj;

    // Puteri configurabile din Panels / Dashboard
    public static double PUTERE_INTAKE = 1.0;
    public static double PUTERE_TRANSFER = 1.0;

    // Praguri pentru detectia de stall pe motorul de transfer (cu encoder + curent)
    public static double PRAG_CURENT_AMPS = 4.5;      // Amperi peste care consideram ca motorul e fortat
    public static double PRAG_VITEZA_TICKS = 150.0;   // Ticks/sec sub care consideram ca motorul aproape s-a blocat
    public static double TIMP_PORNIRE_MS = 250.0;     // Ignora varful de curent la plecarea de pe loc (inrush current)
    public static double TIMP_CONFIRMARE_MS = 100.0;  // Cat timp trebuie sa persiste conditia de stall inainte de oprire
    public static boolean FOLOSESTE_SI_ENCODER = true; // Daca e true, cere si curent mare SI viteza mica; daca e false, se uita doar la curent
    public static boolean OPRESTE_SI_INTAKE = false;   // Opreste si intake-ul cand transferul intra in stall

    private final ElapsedTime timerPornire = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final ElapsedTime timerStall = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private boolean transferPornitAnterior = false;
    private boolean inConditieStall = false;
    private boolean opritAutomatStall = false;

    @Override
    public void init() {
        motorIntake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motorTransfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        servoFaras = hardwareMap.get(Servo.class, "servo_faras");
        servoblocaj = hardwareMap.get(Servo.class, "servo_blocaj");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        // RUN_WITHOUT_ENCODER permite controlul direct prin setPower(),
        // dar citeste in continuare getVelocity() si getCurrentPosition() de pe encoder!
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoFaras.setPosition(0.82);
        servoblocaj.setPosition(0.02);
    }

    @Override
    public void loop() {
        double curentTransfer = motorTransfer.getCurrent(CurrentUnit.AMPS);
        double curentIntake = motorIntake.getCurrent(CurrentUnit.AMPS);
        double vitezaTransfer = Math.abs(motorTransfer.getVelocity()); // ticks / secunda de pe encoder

        boolean cereTragere = gamepad1.right_trigger > 0.05;
        boolean cereScuipare = gamepad1.left_trigger > 0.05;

        // Daca driverul a eliberat trigger-ul sau da inapoi (scuipare), resetam blocajul automat
        if (!cereTragere || cereScuipare) {
            opritAutomatStall = false;
            inConditieStall = false;
        }

        // Pornim cronometrul de pornire in momentul in care incepe tragerea
        if (cereTragere && !transferPornitAnterior) {
            timerPornire.reset();
            inConditieStall = false;
        }
        transferPornitAnterior = cereTragere;

        // Verificam stall-ul doar cand tragem si dupa ce a trecut varful de curent de la pornire
        if (cereTragere && !opritAutomatStall && timerPornire.milliseconds() > TIMP_PORNIRE_MS) {
            boolean curentPreaMare = curentTransfer >= PRAG_CURENT_AMPS;
            boolean vitezaPreaMica = vitezaTransfer <= PRAG_VITEZA_TICKS;

            boolean aproapeInStall = FOLOSESTE_SI_ENCODER
                    ? (curentPreaMare && vitezaPreaMica)
                    : curentPreaMare;

            if (aproapeInStall) {
                if (!inConditieStall) {
                    timerStall.reset();
                    inConditieStall = true;
                } else if (timerStall.milliseconds() >= TIMP_CONFIRMARE_MS) {
                    opritAutomatStall = true;
                    gamepad1.rumble(300); // Vibratie scurta ca sa simti ca s-a oprit automat
                }
            } else {
                inConditieStall = false;
            }
        }

        // Actionarea motoarelor
        if (cereScuipare) {
            motorIntake.setPower(-PUTERE_INTAKE);
            motorTransfer.setPower(-PUTERE_TRANSFER);
        } else if (cereTragere) {
            if (opritAutomatStall) {
                motorTransfer.setPower(0.0);
                motorIntake.setPower(OPRESTE_SI_INTAKE ? 0.0 : PUTERE_INTAKE);
            } else {
                motorIntake.setPower(PUTERE_INTAKE);
                motorTransfer.setPower(PUTERE_TRANSFER);
            }
        } else {
            motorIntake.setPower(0.0);
            motorTransfer.setPower(0.0);
        }

        // Control faras din D-Pad
        if (gamepad1.dpad_up) {
            servoFaras.setPosition(0.81);
        } else if (gamepad1.dpad_down) {
            servoFaras.setPosition(0.29);
        }

        // Telemetrie pentru calibrarea pragurilor
        telemetry.addData("STALL DETECTAT (Oprit)", opritAutomatStall);
        telemetry.addData("Transfer Curent (A)", "%.2f A (prag: %.2f A)", curentTransfer, PRAG_CURENT_AMPS);
        telemetry.addData("Transfer Viteza (ticks/s)", "%.1f (prag: %.1f)", vitezaTransfer, PRAG_VITEZA_TICKS);
        telemetry.addData("Transfer Pozitie Encoder", motorTransfer.getCurrentPosition());
        telemetry.addData("Intake Curent (A)", "%.2f A", curentIntake);
        telemetry.update();
    }
}
