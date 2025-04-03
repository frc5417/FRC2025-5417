package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivebase.DriveBase;

/**
 * Class for choosing an autonomous command in Elastic, Shuffleboard, SmartDashboard, etc.
 */
public class AutonSelect {
    /* Subsystems */
    private final DriveBase m_driveBase;
    private final Elevator m_elevator;
    private final CoralIntake m_coral;

    /* Auton Select */
    private SendableChooser<Command> autoSelection;
    private final boolean is2025 = true;

    public AutonSelect(DriveBase driveBase, Elevator elevator, CoralIntake coral) {
        /* Subsystems */
        m_driveBase = driveBase;
        m_elevator = elevator;
        m_coral = coral;

        /* Pathplanner */
        try {
            // Pathplanner configured
            autoSelection = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> is2025 ? stream.filter(auto -> auto.getName().startsWith("2025"))
                  : stream
              );
        } catch (RuntimeException e) {
            // No pathplanner configured
            e.printStackTrace();

            /* Set auton to none */
            autoSelection = new SendableChooser<>();
            autoSelection.setDefaultOption("None", Commands.none());
        }
        autoSelection.addOption("Leave Auto (Timed)", new LeaveAuto(m_driveBase));
        autoSelection.addOption("L4 Center (Timed)", new L4Auton(m_driveBase, m_elevator, m_coral));
        initTelemetry();
    }
    
    public Command getAuto() {
        return autoSelection.getSelected();
    }
    
    public void initTelemetry() {
        SmartDashboard.putData("Auton Chooser", autoSelection);
    }

}
