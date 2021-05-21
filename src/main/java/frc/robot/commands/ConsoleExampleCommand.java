package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.strykeforce.console.ConsoleSubsystem;
import org.strykeforce.console.Font;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.measurable.Measure;

public class ConsoleExampleCommand extends SequentialCommandGroup {

  private final ConsoleSubsystem consoleSubsystem;
  private int counter = 0;

  public ConsoleExampleCommand(ConsoleSubsystem consoleSubsystem, DriveSubsystem driveSubsystem) {
    this.consoleSubsystem = consoleSubsystem;
    List<ConsoleMeasure> measures = new ArrayList<>();
    var modules = driveSubsystem.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      var module = (TalonSwerveModule) modules[i];
      var azimuthTalon = module.getAzimuthTalon();
      var azimuthName = String.format("azimuth %d", i);
      measures.add(new ConsoleMeasure(new Measure(azimuthName,
          () -> azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF)));
      measures.add(new ConsoleMeasure(new Measure(azimuthName,
          () -> ((int) azimuthTalon.getSelectedSensorPosition()) & 0xFFF)));
    }
    addRequirements(consoleSubsystem);
    double timeout = 5.0;
    addCommands(
        new StartEndCommand(() -> consoleSubsystem.writeStringCentered("Stryke Force", 5
        ),
            consoleSubsystem::clear).withTimeout(timeout),
        new StartEndCommand(() -> {
          consoleSubsystem.writeStringCentered("Counter", 5);
          consoleSubsystem.writeString(String.format("counter = %d", counter), 0, 18, false);
          consoleSubsystem.writeString(String.format("counter = %d", ++counter), 0, 18);
        }, consoleSubsystem::clear).withTimeout(timeout),
        new StartEndCommand(() -> {
          consoleSubsystem.writeStringCentered("Azimuth Pos.", 5);
          int y = 2 * Font.FONT_5X8.getOuterHeight();
          for (int i = 0; i < 4; i++) {
            var m = measures.get(2 * i + 1);
            var line = String.format("%s = %d", m.measure.getName(),
                (int) m.measure.getMeasurement().getAsDouble());
            consoleSubsystem.writeString(m.previous, 2, y, false);
            consoleSubsystem.writeString(line, 2, y);
            m.previous = line;
            y += Font.FONT_5X8.getOuterHeight();
          }
        }, consoleSubsystem::clear).withTimeout(timeout),
        new StartEndCommand(() -> {
          consoleSubsystem.writeStringCentered("Azimuth Abs. Pos.", 5);
          int y = 2 * Font.FONT_5X8.getOuterHeight();
          for (int i = 0; i < 4; i++) {
            var m = measures.get(2 * i);
            var line = String.format("%s = %d", m.measure.getName(),
                (int) m.measure.getMeasurement().getAsDouble());
            consoleSubsystem.writeString(m.previous, 2, y, false);
            consoleSubsystem.writeString(line, 2, y);
            m.previous = line;
            y += Font.FONT_5X8.getOuterHeight();
          }
        }, consoleSubsystem::clear).withTimeout(timeout)
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    consoleSubsystem.clear();
  }

  private static class ConsoleMeasure {

    Measure measure;
    String previous = "";

    public ConsoleMeasure(Measure measure) {
      this.measure = measure;
    }
  }
}
