package constantseditor;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Reads and writes numeric {@code public static final} fields in {@code Constants.java} while
 * preserving comments and derived lines (e.g. {@code INTAKING_LOADER_OUTPUT = ... / 12.0}).
 */
final class ConstantsJavaFile {

  /** Matches numbered archives; both extensions so legacy {@code .java} archives still advance the counter. */
  private static final Pattern ARCHIVE_FILE_NUM =
      Pattern.compile("^Constants_archive_(\\d+)\\.(?:java|txt)$");

  private static final Pattern LINE =
      Pattern.compile(
          "^(\\s*)public static final (int|double) (\\w+) = ([^;]+)(;.*)$", Pattern.MULTILINE);

  private ConstantsJavaFile() {}

  static Map<String, String> readRhsByName(Path file) throws IOException {
    String content = Files.readString(file, StandardCharsets.UTF_8);
    Map<String, String> out = new HashMap<>();
    Matcher m = LINE.matcher(content);
    while (m.find()) {
      out.put(m.group(3), m.group(4).trim());
    }
    return out;
  }

  /**
   * Replaces the RHS for a known constant in the given source string. {@code newRhs} is inserted
   * as-is (number or identifier expression).
   */
  static String replaceRhsInContent(String content, String name, String javaType, String newRhs)
      throws IOException {
    Pattern one =
        Pattern.compile(
            "^(\\s*)public static final "
                + Pattern.quote(javaType)
                + " "
                + Pattern.quote(name)
                + " = ([^;]+)(;.*)$",
            Pattern.MULTILINE);
    Matcher matcher = one.matcher(content);
    if (!matcher.find()) {
      throw new IOException("Could not find: public static final " + javaType + " " + name);
    }
    String repl =
        matcher.group(1)
            + "public static final "
            + javaType
            + " "
            + name
            + " = "
            + newRhs
            + matcher.group(3);
    return matcher.replaceFirst(Matcher.quoteReplacement(repl));
  }

  record RhsPatch(String name, String javaType, String rhs) {}

  /**
   * Archive directory at the <strong>project root</strong> ({@code constantsArchive/}), found by
   * walking up from {@code Constants.java} until a directory contains {@code src/main/java}. If
   * that layout is not found, falls back to {@code constantsArchive} next to the {@code frc/robot}
   * package (legacy).
   */
  static Path archiveDirectoryFor(Path constantsJava) {
    Path dir = constantsJava.toAbsolutePath().normalize().getParent();
    while (dir != null) {
      if (Files.isDirectory(dir.resolve("src/main/java"))) {
        return dir.resolve("constantsArchive");
      }
      dir = dir.getParent();
    }
    return constantsJava.getParent().resolve("constantsArchive");
  }

  /**
   * Moves the current {@code Constants.java} to {@code
   * <project-root>/constantsArchive/Constants_archive_NNNN.txt} (incrementing {@code NNNN}; {@code
   * .txt} avoids compiling archived sources), then writes patched content to a new {@code
   * Constants.java} at the original path.
   *
   * @return path to the archived file
   */
  static Path saveWithArchive(Path constantsFile, List<RhsPatch> patches) throws IOException {
    Path archiveDir = archiveDirectoryFor(constantsFile);
    Files.createDirectories(archiveDir);
    Path archiveFile = nextArchivePath(archiveDir);

    String content = Files.readString(constantsFile, StandardCharsets.UTF_8);
    Files.move(constantsFile, archiveFile, StandardCopyOption.REPLACE_EXISTING);

    for (RhsPatch p : patches) {
      content = replaceRhsInContent(content, p.name(), p.javaType(), p.rhs());
    }
    Files.writeString(constantsFile, content, StandardCharsets.UTF_8);
    return archiveFile;
  }

  static Path nextArchivePath(Path archiveDir) throws IOException {
    int max = 0;
    if (Files.isDirectory(archiveDir)) {
      try (Stream<Path> stream = Files.list(archiveDir)) {
        for (Path p : stream.toList()) {
          if (!Files.isRegularFile(p)) {
            continue;
          }
          Matcher m = ARCHIVE_FILE_NUM.matcher(p.getFileName().toString());
          if (m.matches()) {
            max = Math.max(max, Integer.parseInt(m.group(1)));
          }
        }
      }
    }
    return archiveDir.resolve(String.format("Constants_archive_%04d.txt", max + 1));
  }
}
