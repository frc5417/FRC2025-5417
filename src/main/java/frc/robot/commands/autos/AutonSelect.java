package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Class for choosing an autonomous command in Elastic, Shuffleboard, SmartDashboard, etc.
 */
public class AutonSelect {
    private final SendableChooser<Command> autoSelection;

    public AutonSelect() {
        autoSelection = AutoBuilder.buildAutoChooser();
        initTelemetry();
    }
    
    public Command getAuto() {
        return autoSelection.getSelected();
    }
    
    public void initTelemetry() {
        SmartDashboard.putData("Auton Chooser", autoSelection);
    }

}
