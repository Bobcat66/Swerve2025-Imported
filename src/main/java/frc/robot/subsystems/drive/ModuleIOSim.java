package frc.robot.subsystems.drive;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;

public class ModuleIOSim implements ModuleIO {
    private final DCMotor neoSim = DCMotor.getNEO(1);

    private final SparkMax m_turnMotor;
    private final SparkMaxSim m_turnSim;

    private final SparkMax m_driveMotor;
    private final SparkMaxSim m_driveSim;
}
