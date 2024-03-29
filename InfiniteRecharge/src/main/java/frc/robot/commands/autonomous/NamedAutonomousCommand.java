/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public class NamedAutonomousCommand {
    public Command Command;
    public String Name;

    public NamedAutonomousCommand(String name, Command command) {
        this.Name = name;
        this.Command = command;
    }
}
