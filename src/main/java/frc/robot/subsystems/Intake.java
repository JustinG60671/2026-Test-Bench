package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private TalonFX intakeMotor;
    private final CANBus canbus = new CANBus(Constants.Intake.CANBUS);
    public Intake() {
        //create config object 
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_CURRENT_LIMIT, canbus);

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public enum IntakeStates {
        // Inward, off, outward
        IN,
        OUT,
        OFF,
    }
    public void setIntake(IntakeStates state) {
        switch (state) {
            case IN:
                setRawPower(0.01);
                break;
            case OUT:
                setRawPower(-0.01);
                break;
            case OFF:
                setRawPower(0);
                break;
        }
    }

    public void setRawPower(double power) {
        intakeMotor.set(power);
    }

}
