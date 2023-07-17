package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class SetRotation extends CommandBase{
  CANSparkMax 
  public SetRotation(Pivot pivot) {

    addRequirements(pivot);
  }
}
