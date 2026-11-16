package org.firstinspires.ftc.teamcode.Module;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class IntakeModule {
    private final HardwareMap hardwareMap;
    private DcMotorEx mi, mtr;
    private Servo servo_faras;
    private VoltageSensor voltageSensor;

    // Compensare voltaj pentru motorul de transfer
    public static boolean enableVoltageCompensationTransfer = true;
    public static double nominalVoltageTransfer = 10.5; // Voltajul nominal de referinta (configurabil in Panels)
    public static double transferVoltage = 10.5;

    // Protectie la stall pentru motorul de transfer (curent + encoder)
    public static boolean PROTECTIE_STALL_ACTIVA = true;
    public static double PRAG_CURENT_AMPS = 4.5;      // Amperi peste care consideram ca motorul e fortat
    public static double PRAG_VITEZA_TICKS = 150.0;   // Ticks/sec sub care consideram ca motorul aproape s-a blocat
    public static double TIMP_PORNIRE_MS = 250.0;     // Ignora varful de curent la plecarea de pe loc
    public static double TIMP_CONFIRMARE_MS = 100.0;  // Cat timp trebuie sa persiste conditia de stall
    public static boolean FOLOSESTE_SI_ENCODER = true; // True = cere si curent mare SI viteza mica pe encoder
    public static boolean OPRESTE_SI_INTAKE = false;   // False = opreste doar transferul, intake-ul merge in continuare

    public static double curentTransfer = 0.0;
    public static double vitezaTransfer = 0.0;
    public static boolean opritAutomatStall = false;

    private final ElapsedTime timerPornire = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final ElapsedTime timerStall = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean transferPornitAnterior = false;
    private boolean inConditieStall = false;

    private double lastIntakePower = 0.0;
    private double lastTransferPower = 0.0;
    private double lastFarasPosition = -1.0;

    private static final double POWER_THRESHOLD = 0.01;

    public IntakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        mi = hardwareMap.get(DcMotorEx.class, "motor_intake");
        mtr = hardwareMap.get(DcMotorEx.class, "motor_transfer");
        servo_faras = hardwareMap.get(Servo.class, "servo_faras");

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        mi.setDirection(DcMotorSimple.Direction.FORWARD);

        mi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setFarasPosition(0.82);
        resetStall();
    }

    public void resetStall() {
        opritAutomatStall = false;
        inConditieStall = false;
        transferPornitAnterior = false;
    }

    private void verificaStallTransfer() {
        if (!PROTECTIE_STALL_ACTIVA) {
            opritAutomatStall = false;
            inConditieStall = false;
            return;
        }

        if (!transferPornitAnterior) {
            timerPornire.reset();
            inConditieStall = false;
            transferPornitAnterior = true;
        }

        curentTransfer = mtr.getCurrent(CurrentUnit.AMPS);
        vitezaTransfer = Math.abs(mtr.getVelocity());

        if (!opritAutomatStall && timerPornire.milliseconds() > TIMP_PORNIRE_MS) {
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
                }
            } else {
                inConditieStall = false;
            }
        }
    }

    public void setIntakePower(double power) {
        if (Math.abs(power) < POWER_THRESHOLD) {
            power = 0.0;
        }

        if (Math.abs(power - lastIntakePower) > POWER_THRESHOLD || (power == 0.0 && lastIntakePower != 0.0)) {
            mi.setPower(power);
            lastIntakePower = power;
        }
    }

    public void setTransferPower(double power) {
        if (Math.abs(power) < POWER_THRESHOLD) {
            power = 0.0;
        }

        double targetPower = power;
        if (power != 0.0 && enableVoltageCompensationTransfer && voltageSensor != null) {
            try {
                transferVoltage = voltageSensor.getVoltage();
                if (transferVoltage > 6.0) {
                    targetPower = Range.clip(power * (nominalVoltageTransfer / transferVoltage), -1.0, 1.0);
                }
            } catch (Exception ignored) {
            }
        }

        if (Math.abs(targetPower - lastTransferPower) > POWER_THRESHOLD || (targetPower == 0.0 && lastTransferPower != 0.0)) {
            mtr.setPower(targetPower);
            lastTransferPower = targetPower;
        }
    }

    public void trage_intake(double power) {
        if (opritAutomatStall && OPRESTE_SI_INTAKE) {
            setIntakePower(0.0);
        } else {
            setIntakePower(Math.abs(power));
        }
    }

    public void scuipa_intake(double power) {
        setIntakePower(-Math.abs(power));
    }

    public void stop_intake() {
        setIntakePower(0.0);
    }

    public void trage_transfer(double power) {
        verificaStallTransfer();
        if (opritAutomatStall) {
            setTransferPower(0.0);
        } else {
            setTransferPower(Math.abs(power));
        }
    }

    // Folosit in timpul tragerii in Hive (fara oprire de stall, ca sa impinga toate bilele in shooter)
    public void trage_transfer_tragere(double power) {
        resetStall();
        setTransferPower(Math.abs(power));
    }

    public void scuipa_transfer(double power) {
        resetStall();
        setTransferPower(-Math.abs(power));
    }

    public void stop_transfer() {
        resetStall();
        setTransferPower(0.0);
    }

    public void setFarasPosition(double targetPosition) {
        if (Math.abs(targetPosition - lastFarasPosition) > 0.005) {
            servo_faras.setPosition(targetPosition);
            lastFarasPosition = targetPosition;
        }
    }

    public void farassus() {
        setFarasPosition(0.81);
    }

    public void farasjos() {
        setFarasPosition(0.29);
    }
}
