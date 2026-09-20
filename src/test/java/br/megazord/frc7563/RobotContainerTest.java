// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Instantiation test for {@link RobotContainer}.
 *
 * <p>
 * Desktop JUnit tests run with {@code RobotBase.isReal() == false}, so {@link
 * Constants.RobotConstants#robotMode} resolves to {@code SIM} and construction goes through
 * the {@code SwerveModuleIOSim} / {@code GyroIOSim} branch - no roboRIO, CAN bus or real
 * joystick required. This just proves the container wires up (subsystems, default commands,
 * button bindings) without throwing, which is the fastest possible check that a change hasn't
 * broken robot boot before anyone deploys to hardware.
 */
class RobotContainerTest {

  @BeforeEach
  void setupHal() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    CommandScheduler.getInstance().cancelAll();
  }

  @AfterEach
  void teardownHal() {
    CommandScheduler.getInstance().cancelAll();
    HAL.shutdown();
  }

  @Test
  void constructsWithoutThrowing() {
    assertDoesNotThrow(RobotContainer::new, "RobotContainer() threw during construction");
  }

  /* 
  @Test
  void ledSubsystemIsWiredUpAfterConstruction() {
    new RobotContainer();

    // ledSubsystem is populated as part of the constructor (after the swerve drive is built),
    // so a null value here means construction didn't run to completion even though it didn't
    // throw.
    assertNotNull(RobotContainer.ledSubsystem, "ledSubsystem was not initialized by RobotContainer()");
  }

  @Test
  void driverJoystickIsBoundToPortZero() {
    RobotContainer container = new RobotContainer();

    assertNotNull(container.driverJoystick);
    assertDoesNotThrow(() -> container.driverJoystick.getHID().getPort());
  }

  /*@Test
  void getAutonomousCommandDoesNotThrow() {
    RobotContainer container = new RobotContainer();

    // Currently returns null (no autonomous routine wired up yet) - this test documents that
    // and will fail loudly (which is desired) the day someone wires up an auto and forgets to
    // update this assumption, rather than silently passing either way.
    assertDoesNotThrow(container::getAutonomousCommand);
    assertNull(container.getAutonomousCommand());
  }*/
}
