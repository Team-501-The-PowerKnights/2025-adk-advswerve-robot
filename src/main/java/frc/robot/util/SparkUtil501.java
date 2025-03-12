/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This class contains the implementation of the <code>Lift</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @version 2025.0.0
 */
package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import java.util.function.Supplier;

public class SparkUtil501 {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;
  /** */
  public static REVLibError sparkStickyError = REVLibError.kOk;

  /** Attempts to run the command until no error is produced. */
  public static REVLibError tryUntilOk(
      SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    REVLibError error = REVLibError.kOk;
    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
        sparkStickyError = error;
      }
    }

    return error;
  }

  /** Attempts to run the command until no error is produced. */
  public static REVLibError tryUntilOk(
      RelativeEncoder encoder, int maxAttempts, Supplier<REVLibError> command) {
    REVLibError error = REVLibError.kOk;
    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
        sparkStickyError = error;
      }
    }
    return error;
  }
}
