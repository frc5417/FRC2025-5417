package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Controllers {
    private static final CommandXboxController m_driver = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
    private static final CommandXboxController m_manipulator = new CommandXboxController(Constants.ControllerConstants.kManipulatorControllerPort);

    /**
     * Methods which relate to the driver controller.
     */
    public static class DriverInput {
        //
        // Joystick Inputs
        //
        public static double getLeftJoyX() {
            double input = m_driver.getLeftX();
            if (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) {
                return input;
            } else {
                return 0;
            }
        }

        public static double getRightJoyX() {
            double input = m_driver.getRightX();
            if (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) {
                return input;
            } else {
                return 0;
            }
        }

        public static double getRightJoyY() {
            double input = m_driver.getRightY();
            if (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) {
                return input;
            } else {
                return 0;
            }
        }

        //
        // Button Inputs
        //
        public static boolean getA() {
            return m_manipulator.a().getAsBoolean();
        }
    }

    /**
     * Methods which relate to the manipulator controller.
     */
    public static class ManipulatorInput {
        //
        // Joystick Inputs
        //
        public static double getRightJoyY() {
            double input = m_manipulator.getRightY();
            if (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) {
                return input;
            } else {
                return 0;
            }

        }
        
        //
        // Button Inputs
        //
        public static boolean getA() {
            return m_manipulator.a().getAsBoolean();
        }

        public static boolean getB() {
            return m_manipulator.b().getAsBoolean();
        }

        public static boolean getY() {
            return m_manipulator.y().getAsBoolean();
        }

        public static boolean getX() {
            return m_manipulator.a().getAsBoolean();
        }

        public static boolean getDpadRight() {
            return m_manipulator.povRight().getAsBoolean();
        }

        //
        // Trigger Inputs
        //
        public static double getRightTrigger() {
            double input = m_manipulator.getRightTriggerAxis();
            if (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) {
                return input;
            } else {
                return 0;
            }
        }

        public static double getLeftTrigger() {
            double input = m_manipulator.getLeftTriggerAxis();
            if (Math.abs(input) > Constants.ControllerConstants.joystickDeadband) {
                return input;
            } else {
                return 0;
            }
        }
    }
}
