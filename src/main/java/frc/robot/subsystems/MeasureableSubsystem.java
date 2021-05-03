package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.jetbrains.annotations.NotNull;
import org.strykeforce.thirdcoast.telemetry.item.Measurable;
import org.strykeforce.thirdcoast.telemetry.item.Measure;

public abstract class MeasureableSubsystem extends SubsystemBase implements Measurable {

  @NotNull
  @Override
  public String getDescription() {
    return getName();
  }

  /**
   * Provide a unique subsystem identification number for the Telemetry system.
   * @return unique subsystem number.
   */
  public int getSubsystemNumber() {
    return getDescription().hashCode();
  }

  @Override
  public int getDeviceId() {
    return getSubsystemNumber();
  }

  @NotNull
  @Override
  public String getType() {
    return "subsystem";
  }

  @Override
  public int compareTo(@NotNull Measurable other) {
    var result = getType().compareTo(other.getType());
    if (result != 0) {
      return result;
    }
    return Integer.compare(getDeviceId(), other.getDeviceId());
  }

}
