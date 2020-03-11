/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations;
import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;

public class DriveAutonomousCommand {
    public static Command GenerateCommand() {

        return new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD);
    }
}
