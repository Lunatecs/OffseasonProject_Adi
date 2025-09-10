package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ElevatorVisualizer;

public class ElevatorSubSystem extends SubsystemBase {

    private TalonFX motor1;
    private TalonFX motor2;
    private DigitalInput elevatorLimitSwitch;

    private final double gearRatio = 0.25;
    private final double spoolDiameter = 2.0;
    private final double rotationToInches = spoolDiameter * Math.PI * gearRatio;
    private final double initialElevatorHeight = 5.75;

    private double simulatedHeight = initialElevatorHeight; // for simulation
    private final ElevatorVisualizer visualizer = new ElevatorVisualizer();

    public ElevatorSubSystem() {
        if (RobotBase.isReal()) {
            // Real hardware
            motor1 = new TalonFX(Constants.ElevatorSubSystemConstants.CAN_ID_MOTOR1);
            motor2 = new TalonFX(Constants.ElevatorSubSystemConstants.CAN_ID_MOTOR2);
            motor1.setNeutralMode(NeutralModeValue.Brake);
            motor2.setNeutralMode(NeutralModeValue.Brake);
            motor1.setPosition(0);
            motor2.setControl(new Follower(motor1.getDeviceID(), true));

            elevatorLimitSwitch = new DigitalInput(Constants.ElevatorSubSystemConstants.INPUT_ID);
        } else {
            // Simulation / AdvantageScope only
            motor1 = null;
            motor2 = null;
            elevatorLimitSwitch = null;
        }
    }

    public double getInitialElevatorHeight() {
        return initialElevatorHeight;
    }

    public void setSpeed(double speed) {
        if (RobotBase.isReal()) {
            motor1.set(speed);
        } else {
            // simulate movement in software
            simulatedHeight += speed * 0.02; // 20ms loop
        }
    }

    public boolean getLimitSwitch() {
        if (RobotBase.isReal()) {
            return !elevatorLimitSwitch.get();
        } else {
            return simulatedHeight <= initialElevatorHeight; // simulated limit
        }
    }

    public double getEncoder() {
        if (RobotBase.isReal()) {
            return motor1.getPosition().getValueAsDouble();
        } else {
            return (simulatedHeight - initialElevatorHeight) / rotationToInches;
        }
    }

    public double getElevatorHeight() {
        if (RobotBase.isReal()) {
            return (getEncoder() * rotationToInches) + initialElevatorHeight;
        } else {
            return simulatedHeight;
        }
    }

    @Override
    public void periodic() {
        if (getLimitSwitch() && getEncoder() != 0 && RobotBase.isReal()) {
            motor1.setPosition(0);
        }
        SmartDashboard.putNumber("elevator height", getElevatorHeight());

        visualizer.update(getElevatorHeight());
    }
}
