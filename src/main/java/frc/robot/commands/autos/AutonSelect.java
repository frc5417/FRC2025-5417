package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Class for choosing an autonomous command in Elastic, Shuffleboard, SmartDashboard, etc.
 */
public class AutonSelect {
    private SendableChooser<Command> autoSelection;
    private final boolean is2025 = true;

    public AutonSelect() {
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
        initTelemetry();
    }
    
    public Command getAuto() {
        return autoSelection.getSelected();
    }
    
    public void initTelemetry() {
        SmartDashboard.putData("Auton Chooser", autoSelection);
    }

}
