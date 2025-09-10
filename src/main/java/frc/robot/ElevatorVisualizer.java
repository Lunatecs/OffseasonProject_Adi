package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import org.littletonrobotics.junction.Logger;




public class ElevatorVisualizer {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevatorLigament;

    public ElevatorVisualizer() {
        mech = new Mechanism2d(60, 100); // width, height in pixels
        root = mech.getRoot("Base", 30, 5); // root positioned near bottom
        elevatorLigament = root.append(new MechanismLigament2d("Elevator", 0, 90));

        // 🔑 Immediately publish a starting state so AdvantageScope sees the key
        SmartDashboard.putData("Mechanism2d/Elevator", mech);
    }

    public void update(double elevatorHeightMeters) {
        elevatorLigament.setLength(elevatorHeightMeters);
        SmartDashboard.putData("Mechanism2d/Elevator", mech);
        Logger.recordOutput("Elevator/HeightMeters", elevatorHeightMeters);

    }
}
