package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.console.Font;
import org.strykeforce.console.SSD1306;
import org.strykeforce.telemetry.measurable.Measure;

public class ConsoleSubsystem extends SubsystemBase {

  private static final Logger logger = LoggerFactory.getLogger(ConsoleSubsystem.class);
  private final List<ConsoleMeasure> measures = new ArrayList<>();
  SSD1306 console = new SSD1306();
  int counter = 0;
  private final List<Runnable> screens = List.of(
      () -> {
        console.drawStringCentered("Stryke Force", Font.FONT_5X8, 5, true);
      },
      () -> {
        console.drawStringCentered("Counter", Font.FONT_5X8, 5, true);
        console.drawString(String.format("counter = %d", counter), Font.FONT_5X8, 0, 18, false);
        console.drawString(String.format("counter = %d", ++counter), Font.FONT_5X8, 0, 18, true);
      },
      () -> {
        console.drawStringCentered("Azimuth Abs. Pos.", Font.FONT_5X8, 5, true);
        int y = 2 * Font.FONT_5X8.getOuterHeight();
        for (var m : measures) {
          var line = String.format("%s = %d", m.measure.getName(),
              (int) m.measure.getMeasurement().getAsDouble());
          console.drawString(m.previous, Font.FONT_5X8, 2, y, false);
          console.drawString(line, Font.FONT_5X8, 2, y, true);
          m.previous = line;
          y += Font.FONT_5X8.getOuterHeight();
        }
      },
      () -> console.clear()
  );
  private int screenIndex = 0;

  public void register(Measure measure) {
    measures.add(new ConsoleMeasure(measure));
  }


  public void toggle() {
    screenIndex++;
    console.clear();
  }

  @Override
  public void periodic() {
    screens.get(screenIndex % screens.size()).run();
    console.periodic();
  }

  public void writeString(String value, int x, int y) {
    console.drawString(value, Font.FONT_5X8, x, y, true);
  }

  public void clear() {
    console.clear();
  }

  private static class ConsoleMeasure {

    Measure measure;
    String previous = "";

    public ConsoleMeasure(Measure measure) {
      this.measure = measure;
    }
  }
}
