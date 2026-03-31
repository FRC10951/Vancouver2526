package constantseditor;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import javax.swing.BorderFactory;
import javax.swing.JComponent;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.filechooser.FileNameExtensionFilter;

/**
 * Windows-friendly Swing GUI to tune {@code frc.robot.Constants} values. Run from project root:
 *
 * <pre>
 *   .\\gradlew :constants-editor:run
 * </pre>
 */
public final class ConstantsEditorApp extends JFrame {

  private static final String TOGGLE_RPM = "SHOOTER_TARGET_SPEED_TOGGLE_RPM";
  private static final String INTAKE_RPM = "SHOOTER_TARGET_SPEED_INTAKE_RPM";

  private final Path defaultConstantsPath;
  private Path currentFile;
  private final JLabel pathLabel;
  private final Map<String, JTextField> fields = new LinkedHashMap<>();
  private final JCheckBox toggleMatchesIntake = new JCheckBox("Toggle shooter RPM same as intake RPM");
  private final JTextField toggleRpmField = new JTextField(8);

  private ConstantsEditorApp(Path defaultConstantsPath) {
    super("Vancouver2526 — Constants.java editor");
    this.defaultConstantsPath = defaultConstantsPath;
    this.currentFile = defaultConstantsPath;

    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setMinimumSize(new Dimension(720, 520));

    JTabbedPane tabs = new JTabbedPane();
    tabs.addTab("Main", scroll(wrapFields(buildMainPanel())));
    tabs.addTab("Drive", scroll(wrapFields(buildDrivePanel())));
    tabs.addTab("IO / Shooter", scroll(wrapFields(buildIoPanel())));
    tabs.addTab("Autonomous", scroll(wrapFields(buildAutoPanel())));
    tabs.addTab("Operator", scroll(wrapFields(buildOperatorPanel())));

    JPanel top = new JPanel(new FlowLayout(FlowLayout.LEFT));
    JButton choose = new JButton("Open Constants.java…");
    choose.addActionListener(e -> chooseFile());
    JButton reload = new JButton("Reload");
    reload.addActionListener(e -> loadFromFile());
    JButton save = new JButton("Save (archives previous revision)");
    save.addActionListener(e -> saveToFile());
    top.add(choose);
    top.add(reload);
    top.add(save);
    top.add(new JLabel("  File: "));
    pathLabel = new JLabel(shortenPath(currentFile));
    pathLabel.setToolTipText(currentFile.toString());
    top.add(pathLabel);

    add(top, BorderLayout.NORTH);
    add(tabs, BorderLayout.CENTER);

    loadFromFile();
  }

  private static JScrollPane scroll(JComponent c) {
    JScrollPane sp = new JScrollPane(c);
    sp.getVerticalScrollBar().setUnitIncrement(16);
    return sp;
  }

  private JPanel wrapFields(JPanel inner) {
    JPanel p = new JPanel(new BorderLayout());
    p.add(inner, BorderLayout.NORTH);
    return p;
  }

  /**
   * High-traffic tuning: shooter/intake RPM, key voltages, driver scaling, and main loop gains. Must
   * be built before other tabs so {@link #field} can reuse the same widgets elsewhere.
   */
  private JPanel buildMainPanel() {
    JPanel p = new JPanel(new GridBagLayout());
    GridBagConstraints g = new GridBagConstraints();
    g.insets = new Insets(2, 6, 2, 6);
    g.anchor = GridBagConstraints.WEST;
    g.gridx = 0;
    g.gridy = 0;

    row(p, g, "Main — quick tune (RPM, outputs, driver feel)", true);
    row(p, g, "Shooter target speeds (RPM)", false);
    field(p, g, "SHOOTER_TARGET_SPEED_INTAKE_RPM", "double", "Shooter RPM (intake shot)");
    field(p, g, "SHOOTER_TARGET_SPEED_SPINUP50_RPM", "double", "Shooter RPM (50% spin-up)");
    field(p, g, "SHOOTER_TARGET_SPEED_LAUNCH_RPM", "double", "Shooter RPM (launch)");
    field(p, g, "SHOOTER_TARGET_SPEED_HIGH_RPM", "double", "Shooter RPM (high / A button)");
    field(p, g, "SHOOTER_TARGET_SPEED_ULTRA_RPM", "double", "Shooter RPM (ultra / long range)");
    toggleRpmRow(p, g);

    row(p, g, "Intake speed & feed (RPM / voltage)", false);
    field(p, g, "INTAKE_TARGET_SPEED_RPM", "double", "Intake target RPM (negative = in)");
    field(p, g, "INTAKING_INTAKE_OUTPUT", "double", "Intaking intake voltage");
    field(p, g, "LOADER_MOTOR_TARGET_VOLTAGE", "double", "Loader target voltage");
    field(p, g, "FLYWHEEL_SPIN_UP_50_VOLTAGE", "double", "50% flywheel hold voltage");

    row(p, g, "Driver feel", false);
    field(p, g, "DRIVE_SCALING", "double", "Drive stick scaling");
    field(p, g, "ROTATION_SCALING", "double", "Rotation stick scaling");
    field(p, g, "DRIVE_DEADBAND", "double", "Drive deadband");

    row(p, g, "Speed loop gains", false);
    g.gridwidth = 2;
    g.gridx = 0;
    p.add(
        new JLabel(
            "<html><i>Shooter and intake <b>PID + feedforward</b> constants are on the"
                + " <b>IO / Shooter</b> tab.</i></html>"),
        g);
    g.gridy++;
    g.gridwidth = 1;

    row(p, g, "Everything else", true);
    g.gridwidth = 2;
    g.gridx = 0;
    JLabel hint =
        new JLabel(
            "<html><i>CAN IDs, current limits, autonomous paths, USB ports, and remaining timing are on"
                + " the other tabs.</i></html>");
    p.add(hint, g);
    g.gridy++;
    g.gridwidth = 1;
    return p;
  }

  private JPanel buildDrivePanel() {
    JPanel p = new JPanel(new GridBagLayout());
    GridBagConstraints g = new GridBagConstraints();
    g.insets = new Insets(2, 6, 2, 6);
    g.anchor = GridBagConstraints.WEST;
    g.gridx = 0;
    g.gridy = 0;
    row(p, g, "Drive — CAN & geometry", true);
    field(p, g, "LEFT_LEADER_ID", "int", "Left leader CAN ID");
    field(p, g, "LEFT_FOLLOWER_ID", "int", "Left follower CAN ID");
    field(p, g, "RIGHT_LEADER_ID", "int", "Right leader CAN ID");
    field(p, g, "RIGHT_FOLLOWER_ID", "int", "Right follower CAN ID");
    field(p, g, "DRIVE_MOTOR_CURRENT_LIMIT", "int", "Drive motor current limit (A)");
    field(p, g, "WHEEL_DIAMETER_METERS", "double", "Wheel diameter (m)");
    field(p, g, "GEAR_RATIO", "double", "Motor-to-wheel gear ratio");
    field(p, g, "INTAKE_WIGGLE_SPEED", "double", "Intake wiggle speed");
    g.gridwidth = 2;
    g.gridx = 0;
    p.add(
        new JLabel(
            "<html><i>DRIVE_QUADRATURE_ENCODERS_WIRED is a <code>boolean</code> in Constants.java"
                + " (not edited here). Set <code>true</code> when quadrature encoders are wired to"
                + " drive Spark data ports.</i></html>"),
        g);
    g.gridy++;
    g.gridwidth = 1;
    return p;
  }

  private JPanel buildIoPanel() {
    JPanel p = new JPanel(new GridBagLayout());
    GridBagConstraints g = new GridBagConstraints();
    g.insets = new Insets(2, 6, 2, 6);
    g.anchor = GridBagConstraints.WEST;
    g.gridx = 0;
    g.gridy = 0;

    row(p, g, "CAN IDs & current limits", true);
    field(p, g, "IO_MOTOR_ID", "int", "IO / flywheel CAN ID");
    field(p, g, "INTAKE_MOTOR_ID", "int", "Intake CAN ID");
    field(p, g, "LOADER_MOTOR_ID", "int", "Loader CAN ID");
    field(p, g, "IO_MOTOR_CURRENT_LIMIT", "int", "IO motor current (A)");
    field(p, g, "INTAKE_MOTOR_CURRENT_LIMIT", "int", "Intake current (A)");
    field(p, g, "LOADER_MOTOR_CURRENT_LIMIT", "int", "Loader current (A)");
    field(p, g, "FLYWHEEL_MOTOR_CURRENT_LIMIT", "int", "Flywheel current (A)");

    row(p, g, "Shooter speeds (RPM) & PID-ish", true);
    JLabel shooterOnMain =
        new JLabel(
            "<html><i>Shooter RPM targets and right-trigger toggle are on the <b>Main</b> tab.</i></html>");
    g.gridwidth = 2;
    g.gridx = 0;
    p.add(shooterOnMain, g);
    g.gridy++;
    g.gridwidth = 1;
    field(p, g, "SHOOTER_TARGET_SPEED_INTAKE_RPM", "double", "Shooter RPM (intake shot)");
    field(p, g, "SHOOTER_TARGET_SPEED_SPINUP50_RPM", "double", "Shooter RPM (50% spin-up)");
    field(p, g, "SHOOTER_TARGET_SPEED_LAUNCH_RPM", "double", "Shooter RPM (launch)");
    field(p, g, "SHOOTER_TARGET_SPEED_HIGH_RPM", "double", "Shooter RPM (high / A button)");
    field(p, g, "SHOOTER_TARGET_SPEED_ULTRA_RPM", "double", "Shooter RPM (ultra / long range)");
    field(p, g, "SHOOTER_PID_KP", "double", "Shooter PID kP");
    field(p, g, "SHOOTER_PID_KI", "double", "Shooter PID kI");
    field(p, g, "SHOOTER_PID_KD", "double", "Shooter PID kD");
    field(p, g, "SHOOTER_FF_KS", "double", "Shooter FF kS (static)");
    field(p, g, "SHOOTER_FF_KV", "double", "Shooter FF kV (V·s/rot)");
    field(p, g, "SHOOTER_FF_KA", "double", "Shooter FF kA (accel)");
    field(p, g, "SHOOTER_PID_INTEGRATOR_MAX", "double", "Shooter integrator max");
    field(p, g, "SHOOTER_SPINUP_THRESHOLD_FRACTION", "double", "Shooter spin-up threshold (0–1)");
    field(p, g, "SHOOTER_MAX_VOLTAGE", "double", "Shooter max voltage");

    row(p, g, "Intake speed control", true);
    field(p, g, "INTAKE_TARGET_SPEED_RPM", "double", "Intake target RPM (negative = in)");
    field(p, g, "INTAKE_PID_KP", "double", "Intake PID kP");
    field(p, g, "INTAKE_PID_KI", "double", "Intake PID kI");
    field(p, g, "INTAKE_PID_KD", "double", "Intake PID kD");
    field(p, g, "INTAKE_FF_KS", "double", "Intake FF kS");
    field(p, g, "INTAKE_FF_KV", "double", "Intake FF kV (V·s/rot)");
    field(p, g, "INTAKE_FF_KA", "double", "Intake FF kA");
    field(p, g, "INTAKE_PID_INTEGRATOR_MAX", "double", "Intake integrator max");
    field(p, g, "INTAKE_SPINUP_THRESHOLD_FRACTION", "double", "Intake spin-up threshold");
    field(p, g, "INTAKE_MAX_VOLTAGE", "double", "Intake max voltage");

    row(p, g, "Open-loop & timing", true);
    field(p, g, "INTAKING_INTAKE_OUTPUT", "double", "Intaking intake voltage");
    field(p, g, "LOADER_MOTOR_TARGET_VOLTAGE", "double", "Loader target voltage (also drives duty via /12)");
    field(p, g, "PREPARING_LOADER_OUTPUT", "double", "Preparing loader output");
    field(p, g, "LAUNCH_SPIN_UP_SECONDS", "double", "Launch spin-up wait (s)");
    field(p, g, "INTAKE_SPIN_UP_SECONDS", "double", "Intake spin-up wait (s)");
    field(p, g, "INTAKE_AUTON_SPIN_UP_SECONDS", "double", "Auton intake spin-up (s)");
    field(p, g, "FLYWHEEL_SPIN_UP_50_VOLTAGE", "double", "50% flywheel hold voltage");
    field(p, g, "INTAKE_PULSE_ON_SECONDS", "double", "Intake pulse ON (s)");
    field(p, g, "INTAKE_PULSE_OFF_SECONDS", "double", "Intake pulse OFF (s)");

    row(p, g, "Note", true);
    g.gridwidth = 2;
    JLabel note =
        new JLabel(
            "<html><i>INTAKING_LOADER_OUTPUT and LAUNCHING_LOADER_OUTPUT stay derived in Java;<br>"
                + "change LOADER_MOTOR_TARGET_VOLTAGE to retune loader duty.</i></html>");
    p.add(note, g);
    g.gridy++;
    g.gridwidth = 1;
    return p;
  }

  private void toggleRpmRow(JPanel p, GridBagConstraints g) {
    g.gridwidth = 1;
    g.gridx = 0;
    JLabel lab = new JLabel("Right-trigger toggle shooter RPM");
    lab.setToolTipText(TOGGLE_RPM);
    p.add(lab, g);
    g.gridx = 1;
    JPanel row = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 0));
    row.add(toggleMatchesIntake);
    row.add(new JLabel("or RPM:"));
    row.add(toggleRpmField);
    toggleMatchesIntake.addActionListener(
        e -> toggleRpmField.setEnabled(!toggleMatchesIntake.isSelected()));
    p.add(row, g);
    g.gridy++;
  }

  private JPanel buildAutoPanel() {
    JPanel p = new JPanel(new GridBagLayout());
    GridBagConstraints g = new GridBagConstraints();
    g.insets = new Insets(2, 6, 2, 6);
    g.anchor = GridBagConstraints.WEST;
    g.gridx = 0;
    g.gridy = 0;
    row(p, g, "AutoConstants — main autonomous (timed AutoDrive segments)", true);
    field(p, g, "AUTO_INITIAL_SHOOT_SECONDS", "double", "Initial shoot (s)");
    field(p, g, "AUTO_FWD1_SPEED", "double", "Forward 1 speed (0–1)");
    field(p, g, "AUTO_FWD1_SECONDS", "double", "Forward 1 duration (s)");
    field(p, g, "AUTO_TURN1_ROTATION", "double", "Turn 1 rotation (-1..1)");
    field(p, g, "AUTO_TURN1_SECONDS", "double", "Turn 1 duration (s)");
    field(p, g, "AUTO_FWD2_SPEED", "double", "Forward 2 speed (0–1)");
    field(p, g, "AUTO_FWD2_SECONDS", "double", "Forward 2 duration (s)");
    field(p, g, "AUTO_TURN2_ROTATION", "double", "Turn 2 rotation (-1..1)");
    field(p, g, "AUTO_TURN2_SECONDS", "double", "Turn 2 duration (s)");
    field(p, g, "AUTO_FWD_INTAKE_SPEED", "double", "Forward+intake speed (0–1)");
    field(p, g, "AUTO_FWD_INTAKE_SECONDS", "double", "Forward+intake deadline (s)");
    field(p, g, "AUTO_TURN3_ROTATION", "double", "Turn 3 rotation (-1..1)");
    field(p, g, "AUTO_TURN3_SECONDS", "double", "Turn 3 duration (s)");
    field(p, g, "AUTO_FINAL_SHOOT_SECONDS", "double", "Final shoot (s)");

    row(p, g, "Dashboard short autos (SendableChooser)", true);
    field(p, g, "CHOOSER_SHOOT_ONLY_SECONDS", "double", "Shoot only (s)");
    field(p, g, "CHOOSER_SIMPLE_FWD_SPEED", "double", "Short forward speed (0–1)");
    field(p, g, "CHOOSER_SIMPLE_FWD_SECONDS", "double", "Short forward (s)");
    field(p, g, "CHOOSER_SIMPLE_REV_SPEED", "double", "Short backward speed (0–1, neg)");
    field(p, g, "CHOOSER_SIMPLE_REV_SECONDS", "double", "Short backward (s)");
    field(p, g, "CHOOSER_SHOOT_THEN_FWD_SHOOT_SECONDS", "double", "Shoot-then-drive shoot (s)");
    field(p, g, "CHOOSER_DRIVE_2S_SPEED", "double", "Drive 2s speed");
    field(p, g, "CHOOSER_DRIVE_2S_SECONDS", "double", "Drive 2s duration (s)");
    return p;
  }

  private JPanel buildOperatorPanel() {
    JPanel p = new JPanel(new GridBagLayout());
    GridBagConstraints g = new GridBagConstraints();
    g.insets = new Insets(2, 6, 2, 6);
    g.anchor = GridBagConstraints.WEST;
    g.gridx = 0;
    g.gridy = 0;
    row(p, g, "OperatorConstants", true);
    field(p, g, "DRIVER_CONTROLLER_PORT", "int", "Driver USB port");
    field(p, g, "OPERATOR_CONTROLLER_PORT", "int", "Operator USB port");
    field(p, g, "DRIVE_SCALING", "double", "Drive stick scaling");
    field(p, g, "ROTATION_SCALING", "double", "Rotation stick scaling");
    field(p, g, "DRIVE_DEADBAND", "double", "Drive deadband");
    field(p, g, "TRIGGER_THRESHOLD", "double", "Trigger \"pressed\" threshold");
    return p;
  }

  private void row(JPanel p, GridBagConstraints g, String title, boolean section) {
    g.gridwidth = 2;
    g.gridx = 0;
    JLabel l = new JLabel(title);
    if (section) {
      l.setBorder(BorderFactory.createEmptyBorder(12, 0, 4, 0));
    }
    p.add(l, g);
    g.gridy++;
    g.gridwidth = 1;
  }

  /**
   * Adds a row for this constant unless it already exists on the Main tab (same {@link JTextField}
   * is shared).
   */
  private void field(JPanel p, GridBagConstraints g, String name, String javaType, String label) {
    if (fields.containsKey(name)) {
      return;
    }
    g.gridx = 0;
    JLabel lab = new JLabel(label);
    lab.setToolTipText(name + " (" + javaType + ")");
    p.add(lab, g);
    g.gridx = 1;
    JTextField tf = new JTextField(14);
    tf.putClientProperty("javaType", javaType);
    tf.putClientProperty("constName", name);
    p.add(tf, g);
    fields.put(name, tf);
    g.gridy++;
  }

  private void chooseFile() {
    JFileChooser fc = new JFileChooser(defaultConstantsPath.getParent().toFile());
    fc.setSelectedFile(defaultConstantsPath.toFile());
    fc.setFileFilter(new FileNameExtensionFilter("Java source", "java"));
    if (fc.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
      currentFile = fc.getSelectedFile().toPath();
      pathLabel.setText(shortenPath(currentFile));
      pathLabel.setToolTipText(currentFile.toString());
      loadFromFile();
    }
  }

  private void loadFromFile() {
    try {
      Map<String, String> rhs = ConstantsJavaFile.readRhsByName(currentFile);
      for (Map.Entry<String, JTextField> e : fields.entrySet()) {
        String v = rhs.get(e.getKey());
        if (v != null) {
          e.getValue().setText(v);
        }
      }
      String toggleRhs = rhs.get(TOGGLE_RPM);
      if (toggleRhs != null) {
        boolean linked = toggleRhs.equals(INTAKE_RPM);
        toggleMatchesIntake.setSelected(linked);
        toggleRpmField.setEnabled(!linked);
        if (linked) {
          String intake = rhs.getOrDefault(INTAKE_RPM, "");
          toggleRpmField.setText(intake);
        } else {
          toggleRpmField.setText(toggleRhs);
        }
      }
      pathLabel.setText(shortenPath(currentFile));
      pathLabel.setToolTipText(currentFile.toString());
      setTitle("Constants — " + shortenPath(currentFile));
    } catch (IOException ex) {
      JOptionPane.showMessageDialog(
          this, ex.getMessage(), "Could not read file", JOptionPane.ERROR_MESSAGE);
    }
  }

  private void saveToFile() {
    try {
      Map<String, String> updates = new LinkedHashMap<>();
      for (Map.Entry<String, JTextField> e : fields.entrySet()) {
        JTextField tf = e.getValue();
        String type = (String) tf.getClientProperty("javaType");
        String raw = tf.getText().trim();
        if (raw.isEmpty()) {
          throw new IOException("Empty value for " + e.getKey());
        }
        validateAndFormat(type, raw, e.getKey());
        updates.put(e.getKey(), raw);
      }

      String toggleRhs;
      if (toggleMatchesIntake.isSelected()) {
        toggleRhs = INTAKE_RPM;
      } else {
        String raw = toggleRpmField.getText().trim();
        validateAndFormat("double", raw, TOGGLE_RPM);
        toggleRhs = raw;
      }
      updates.put(TOGGLE_RPM, toggleRhs);

      List<ConstantsJavaFile.RhsPatch> patches = new ArrayList<>();
      for (Map.Entry<String, String> en : updates.entrySet()) {
        String name = en.getKey();
        JTextField tf = fields.get(name);
        String javaType = tf != null ? (String) tf.getClientProperty("javaType") : "double";
        patches.add(new ConstantsJavaFile.RhsPatch(name, javaType, en.getValue()));
      }
      Path archivedAs = ConstantsJavaFile.saveWithArchive(currentFile, patches);
      JOptionPane.showMessageDialog(
          this,
          "Saved.\nPrevious revision archived as:\n" + archivedAs,
          "OK",
          JOptionPane.INFORMATION_MESSAGE);
    } catch (IOException | NumberFormatException ex) {
      JOptionPane.showMessageDialog(
          this, ex.getMessage(), "Save failed", JOptionPane.ERROR_MESSAGE);
    }
  }

  private static void validateAndFormat(String javaType, String raw, String name)
      throws IOException {
    try {
      if ("int".equals(javaType)) {
        Integer.parseInt(raw);
      } else {
        Double.parseDouble(raw);
      }
    } catch (NumberFormatException e) {
      throw new IOException("Invalid " + javaType + " for " + name + ": " + raw);
    }
  }

  private static String shortenPath(Path p) {
    String s = p.toString();
    int idx = s.indexOf("src\\main\\java");
    if (idx < 0) {
      idx = s.indexOf("src/main/java");
    }
    if (idx > 0) {
      return "…" + s.substring(idx);
    }
    if (s.length() > 60) {
      return "…" + s.substring(s.length() - 58);
    }
    return s;
  }

  public static void main(String[] args) {
    Path projectRoot = Path.of(System.getProperty("user.dir"));
    Path constants =
        projectRoot.resolve("src/main/java/frc/robot/Constants.java");
    if (!java.nio.file.Files.isRegularFile(constants)) {
      // Running from tools/constants-editor via Gradle: user.dir may be subproject dir
      Path fromSub =
          projectRoot
              .resolve("../../src/main/java/frc/robot/Constants.java")
              .normalize();
      if (java.nio.file.Files.isRegularFile(fromSub)) {
        constants = fromSub;
      }
    }

    Path defaultPath = constants;
    if (args.length >= 1) {
      defaultPath = Path.of(args[0]);
    }

    try {
      UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
    } catch (Exception ignored) {
    }

    Path finalDefault = defaultPath;
    SwingUtilities.invokeLater(
        () -> {
          ConstantsEditorApp w = new ConstantsEditorApp(finalDefault);
          w.pack();
          w.setLocationRelativeTo(null);
          w.setVisible(true);
        });
  }
}
