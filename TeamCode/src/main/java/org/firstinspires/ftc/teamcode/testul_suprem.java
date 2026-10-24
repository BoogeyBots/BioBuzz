package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class testul_suprem extends OpMode {

    int rotatii = 0;
    DcMotorEx mi, mtr, st, dr;
    TouchSensor senzor;
    Servo servoshooter, servo_blocaj;
    private Follower follower;


    public static final double POS_INIT_SERVO = 0.28;

    // Variabile pentru controlul tracțiunii (st și dr)
    boolean motoareStangaDreaptaPornite = false;
    boolean butonAApasatAnterior = false;

    // Variabilă pentru reținerea stării butonului Square
    boolean butonSquareApasatAnterior = false;

    // 1. Timer și stare pentru senzorul care acționează servo_shooter
    private ElapsedTime timerSenzorServo = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean esteSenzorulActiv = false;
    private double timpAsteptareSenzorMs = 500; // 500 milisecunde

    // 2. Timer și stări pentru acțiunea de pe butonul SQUARE (servo_blocaj -> 200ms -> pornire motoare)
    private ElapsedTime timerSquare = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean esteInAsteptareSquare = false;
    private double intarzierePornireMotoareMs = 200; // 200 milisecunde întârziere

    // Starea generală a automatizării de pe square (pornit / oprit)
    private boolean workflowSquarePornit = false;
    IntakeModule intake = null;

    @Override
    public void init() {
        mi = hardwareMap.get(DcMotorEx.class, "motorintake");
        mtr = hardwareMap.get(DcMotorEx.class, "motortransfer");
        servoshooter = hardwareMap.get(Servo.class, "servo_shooter");
        st = hardwareMap.get(DcMotorEx.class, "stanga");
        dr = hardwareMap.get(DcMotorEx.class, "dreapta");
        senzor = hardwareMap.get(TouchSensor.class, "senzor");
        intake = new IntakeModule(hardwareMap);

        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");

        intake.init();
        follower = Constants.create(hardwareMap);

        servoshooter.setPosition(POS_INIT_SERVO);
        servo_blocaj.setPosition(0.02);
    }

    @Override
    public void loop() {

        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        follower.update();

        // --- SECȚIUNEA 1: Logică Senzor + Servo Shooter (Prioritate Maximă) ---
        if(senzor.isPressed() && !esteSenzorulActiv) {
            servoshooter.setPosition(0.12);
            timerSenzorServo.reset();
            esteSenzorulActiv = true;
        }

        if (esteSenzorulActiv) {
            mtr.setPower(0.4);
            mi.setPower(-1);

            if (timerSenzorServo.milliseconds() >= timpAsteptareSenzorMs) {
                servoshooter.setPosition(0.28);
                esteSenzorulActiv = false;
            }
        }
        else {
            // --- SECȚIUNEA 2: Controlul normal prin Trigger-e ---
            // Rulează DOAR DACĂ automatizarea de pe Square nu a preluat controlul
            if (!workflowSquarePornit) {
                if(gamepad1.right_trigger > 0.01) {
                    mtr.setPower(0.5);
                    mi.setPower(-1);
                }
                else if(gamepad1.left_trigger > 0.01) {
                    mtr.setPower(-1);
                    mi.setPower(1);
                }
                else {
                    mtr.setPower(0);
                    mi.setPower(0);
                }
            }
        }

        // --- SECȚIUNEA 3: Logică Toggle Butonul A (pentru motoarele st și dr) ---
        if (gamepad1.a && !butonAApasatAnterior) {
            motoareStangaDreaptaPornite = !motoareStangaDreaptaPornite;
        }
        butonAApasatAnterior = gamepad1.a;

        if (motoareStangaDreaptaPornite) {
            dr.setPower(-0.6);
            st.setPower(0.6);
        } else {
            dr.setPower(0);
            st.setPower(0);
        }

        // --- SECȚIUNEA 4: Logică Butonul Square (Servo_blocaj -> Așteptare -> Motoare) ---
        boolean squareEsteApasatAcum = gamepad1.square;

        if (squareEsteApasatAcum && !butonSquareApasatAnterior) {
            workflowSquarePornit = !workflowSquarePornit; // Comutăm starea (Pornit / Oprit)

            if (workflowSquarePornit) {
                // Apăsarea 1: Mutăm servo-ul la 0.3 și pornim cronometrul de 200ms
                servo_blocaj.setPosition(0.3);
                timerSquare.reset();
                esteInAsteptareSquare = true;
            } else {
                // Apăsarea 2 (sau înapoi): Punem servo-ul la 0.02 și oprim motoarele
                servo_blocaj.setPosition(0.02);
                mi.setPower(0);
                mtr.setPower(0);
                esteInAsteptareSquare = false;
            }
        }
        butonSquareApasatAnterior = squareEsteApasatAcum;

        // Verificăm dacă au trecut cele 200 de milisecunde pentru a porni motoarele
        if (esteInAsteptareSquare && workflowSquarePornit) {
            if (timerSquare.milliseconds() >= intarzierePornireMotoareMs) {
                mi.setPower(-1);
                mtr.setPower(1);
                esteInAsteptareSquare = false; // Am pornit motoarele, oprim așteptarea
            }
        }

        if(gamepad1.dpad_down){
            intake.farasjos();
        } else if(gamepad1.dpad_up){
            intake.farassus();
        }

        // --- SECȚIUNEA 5: Telemetry (Pentru ecranul robotului) ---
        telemetry.addData("Senzor Atins:", senzor.isPressed());
        telemetry.addData("Workflow Square Porniți:", workflowSquarePornit);
        telemetry.addData("Putere MTR:", mtr.getPower());
        telemetry.update();
    }
}