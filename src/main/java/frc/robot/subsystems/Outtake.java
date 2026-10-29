package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase{
    private TalonFX outtakeMotor;
    private final CANBus canbus = new CANBus(Constants.Outtake.CANBUS);
    public Outtake() {
        //create config object 
        TalonFXConfiguration outtakeConfig = new TalonFXConfiguration();

        outtakeMotor = new TalonFX(Constants.Outtake.OUTTAKE_MOTOR_CURRENT_LIMIT, canbus);
        outtakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        outtakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        outtakeMotor.getConfigurator().apply(outtakeConfig);
    }

    public enum OuttakeStates {
        // Inward, off, outward
        IN,
        OUT,
        OFF,
    }
    public void setOuttake(OuttakeStates state) {
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
        outtakeMotor.set(power);
    }

}
