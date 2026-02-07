package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.hood.SetHoodAngle;
import frc.robot.commands.hood.ZeroHood;

public class IO {

        public void init(RobotContainer robotContainer) {

                /*
                 * Button bindings live here.
                 *
                 * For readability, we follow two rules:
                 * 1. Trigger variables are named exactly like the button in Keymap.Layout.
                 * 2. Binding code is short and reads like a sentence: "button.onTrue(do thing)"
                 */

                /*
                 * Triggers
                 *
                 * We keep the full set of triggers around (even if they are currently unused)
                 * so it's easy to add bindings later without re-learning the controller map.
                 */

                // Driver controller buttons/triggers (named exactly like Keymap.Layout)
                final Trigger driverAButton = Keymap.Layout.driverAButton;
                final Trigger driverBButton = Keymap.Layout.driverBButton;
                final Trigger driverXButton = Keymap.Layout.driverXButton;
                final Trigger driverYButton = Keymap.Layout.driverYButton;

                final Trigger driverLeftBumper = Keymap.Layout.driverLeftBumper;
                final Trigger driverRightBumper = Keymap.Layout.driverRightBumper;

                final Trigger driverLeftTriggerButton = Keymap.Layout.driverLeftTriggerButton;
                final Trigger driverRightTriggerButton = Keymap.Layout.driverRightTriggerButton;

                final Trigger driverStartButton = Keymap.Layout.driverStartButton;
                final Trigger driverBackButton = Keymap.Layout.driverBackButton;

                final Trigger driverUpButton = Keymap.Layout.driverUpButton;
                final Trigger driverDownButton = Keymap.Layout.driverDownButton;
                final Trigger driverLeftButton = Keymap.Layout.driverLeftButton;
                final Trigger driverRightButton = Keymap.Layout.driverRightButton;

                final Trigger driverLeftStickButton = Keymap.Layout.driverLeftStickButton;
                final Trigger driverRightStickButton = Keymap.Layout.driverRightStickButton;

                // Operator controller buttons/triggers (named exactly like Keymap.Layout)
                final Trigger operatorAButton = Keymap.Layout.operatorAButton;
                final Trigger operatorBButton = Keymap.Layout.operatorBButton;
                final Trigger operatorXButton = Keymap.Layout.operatorXButton;
                final Trigger operatorYButton = Keymap.Layout.operatorYButton;

                final Trigger operatorLeftBumper = Keymap.Layout.operatorLeftBumper;
                final Trigger operatorRightBumper = Keymap.Layout.operatorRightBumper;

                final Trigger operatorLeftTriggerButton = Keymap.Layout.operatorLeftTriggerButton;
                final Trigger operatorRightTriggerButton = Keymap.Layout.operatorRightTriggerButton;

                final Trigger operatorStartButton = Keymap.Layout.operatorStartButton;
                final Trigger operatorBackButton = Keymap.Layout.operatorBackButton;

                final Trigger operatorUpButton = Keymap.Layout.operatorUpButton;
                final Trigger operatorDownButton = Keymap.Layout.operatorDownButton;
                final Trigger operatorLeftButton = Keymap.Layout.operatorLeftButton;
                final Trigger operatorRightButton = Keymap.Layout.operatorRightButton;

                final Trigger operatorLeftStickButton = Keymap.Layout.operatorLeftStickButton;
                final Trigger operatorRightStickButton = Keymap.Layout.operatorRightStickButton;

                /*
                 * Axis-based "virtual buttons" (NOT part of Keymap.Layout, but still Triggers).
                 *
                 * These create a Trigger when a joystick axis crosses a threshold.
                 * Axis indices come from WPILib's Xbox controller mapping.
                 */
                final Trigger operatorAxis1LessThanNeg025 = Keymap.Controllers.operatorController.axisLessThan(1, -0.25);
                final Trigger operatorAxis1GreaterThan025 = Keymap.Controllers.operatorController.axisGreaterThan(1, 0.25);
                final Trigger operatorAxis4GreaterThan025 = Keymap.Controllers.operatorController.axisGreaterThan(4, 0.25);
                final Trigger operatorAxis4LessThanNeg025 = Keymap.Controllers.operatorController.axisLessThan(4, -0.25);

                /*
                 * Hood controls
                 *
                 * - A: re-zero the hood encoder (wherever it is becomes 0 degrees)
                 * - Left bumper: move hood to -10 degrees
                 * - Right bumper: move hood to +10 degrees
                 */
                driverAButton.onTrue(new ZeroHood(robotContainer.hood));
                driverLeftBumper.onTrue(new SetHoodAngle(robotContainer.hood, -10.0));
                driverRightBumper.onTrue(new SetHoodAngle(robotContainer.hood, 10.0));
        }
}
