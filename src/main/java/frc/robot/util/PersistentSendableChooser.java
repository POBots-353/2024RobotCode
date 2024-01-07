// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Preferences;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

public class PersistentSendableChooser<V> implements NTSendable {
  private static final String DEFAULT = "default";

  private static final String SELECTED = "selected";

  private static final String ACTIVE = "active";

  private static final String OPTIONS = "options";

  private static final String INSTANCE = ".instance";

  private final Map<String, V> m_map = new LinkedHashMap<>();

  private String m_defaultChoice = "";
  private final int m_instance;
  private static final AtomicInteger s_instances = new AtomicInteger();

  private final String preferenceName;

  public PersistentSendableChooser(String preferenceName) {
    this.preferenceName = preferenceName;

    String persistentString = Preferences.getString(preferenceName, null);
    if (persistentString != null) {
      m_selected = persistentString;
    }

    m_instance = s_instances.getAndIncrement();
    SendableRegistry.add(this, "SendableChooser", m_instance);
  }

  public void addOption(String name, V object) {
    m_map.put(name, object);
  }

  public void setDefaultOption(String name, V object) {
    m_defaultChoice = name;
    addOption(name, object);
  }

  public String getSelectedName() {
    return m_selected;
  }

  public V getSelected() {
    m_mutex.lock();
    try {
      if (m_selected != null) {
        return m_map.get(m_selected);
      } else {
        return m_map.get(m_defaultChoice);
      }
    } finally {
      m_mutex.unlock();
    }
  }

  private String m_selected;
  private final List<StringPublisher> m_activePubs = new ArrayList<>();
  private final ReentrantLock m_mutex = new ReentrantLock();

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("String Chooser");
    IntegerPublisher instancePub = new IntegerTopic(builder.getTopic(INSTANCE)).publish();
    instancePub.set(m_instance);
    builder.addCloseable(instancePub);
    builder.addStringProperty(DEFAULT, () -> m_defaultChoice, null);
    builder.addStringArrayProperty(OPTIONS, () -> m_map.keySet().toArray(new String[0]), null);
    builder.addStringProperty(
        ACTIVE,
        () -> {
          m_mutex.lock();
          try {
            if (m_selected != null) {
              return m_selected;
            } else {
              return m_defaultChoice;
            }
          } finally {
            m_mutex.unlock();
          }
        },
        null);
    m_mutex.lock();
    try {
      m_activePubs.add(new StringTopic(builder.getTopic(ACTIVE)).publish());
    } finally {
      m_mutex.unlock();
    }
    builder.addStringProperty(
        SELECTED,
        null,
        val -> {
          m_mutex.lock();
          try {
            m_selected = val;
            Preferences.setString(preferenceName, val);
            for (StringPublisher pub : m_activePubs) {
              pub.set(val);
            }
          } finally {
            m_mutex.unlock();
          }
        });
  }
}
