package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.commands.Autos.*;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.A_Star.A_Star;

public class AutonLoader {
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final String[] pathFileName = {"LeaveAuto","LeaveCoralLeft"};

    public AutonLoader(DriveBase driveBase, Vision vision) {
        // photon.setDriverMode(false); 
        A_Star.rectangularObstacle(Constants.Auton.BlueObstacle_TopLeft, Constants.Auton.BlueObstacle_BottomRight);
        A_Star.rectangularObstacle(Constants.Auton.RedObstacle_TopLeft, Constants.Auton.RedObstacle_BottomRight);
        
        RobotContainer.defineNamedCommands();

        m_chooser.addOption("None", Commands.none());

        for (int i = 0; i < pathFileName.length; i++) {
            String x = pathFileName[i];
            try{
                // Load the path you want to follow using its name in the GUI
                PathPlannerPath path = PathPlannerPath.fromPathFile(x);
                
                // Create a path following command using AutoBuilder. This will also trigger event markers.
                Command leavePath = AutoBuilder.followPath(path);
                m_chooser.addOption(x, leavePath);
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            }
        }

        m_chooser.setDefaultOption("None", Commands.none());
        SmartDashboard.putData("Autonomous", m_chooser);
        // SmartDashboard.updateValues();
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAuton() {
        return m_chooser.getSelected();
    }    
}