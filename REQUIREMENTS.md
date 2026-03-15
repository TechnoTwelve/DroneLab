# Requirements & Setup Guide

Everything you need to build and run DroneLab from scratch.

---

## System Requirements

| Requirement | Minimum | Recommended |
|---|---|---|
| **Java** | 17 | 21 LTS |
| **Maven** | 3.6 | 3.9+ |
| **RAM** | 512 MB | 1 GB |
| **OS** | macOS, Linux, Windows 10+ | Any of these |
| **Screen** | 1280 × 800 | 1920 × 1080 (multi-panel GUI) |

Java 17 is required for the `sealed` and `record` types used internally and to
match the `<release>17</release>` compiler target in `pom.xml`.

---

## Installing Prerequisites

### Java 17+

**macOS (Homebrew):**
```bash
brew install openjdk@21
# Follow the brew post-install symlink instructions if needed
java -version   # should print 21.x.x
```

**Linux (apt):**
```bash
sudo apt update
sudo apt install openjdk-21-jdk
java -version
```

**Windows:**
Download and run the installer from [adoptium.net](https://adoptium.net) (Eclipse
Temurin 21 LTS). Tick "Set JAVA_HOME" in the installer.

### Maven 3.6+

**macOS (Homebrew):**
```bash
brew install maven
mvn -version
```

**Linux (apt):**
```bash
sudo apt install maven
mvn -version
```

**Windows:**
Download the binary zip from [maven.apache.org](https://maven.apache.org/download.cgi),
unzip, and add `bin/` to your `PATH`. Set `MAVEN_HOME` to the unzipped directory.

> **Tip:** Both Java and Maven come bundled with **IntelliJ IDEA** — if you use
> IntelliJ you can skip the above and let it manage the JDK automatically.

---

## Getting the Code

```bash
git clone https://github.com/TechnoTwelve/drone-lab.git
cd drone-lab
```

---

## Building

```bash
# Compile and package an executable JAR (skips tests for speed)
mvn package -DskipTests

# Build and run all 265 tests
mvn test
```

The JAR is created at `target/drone-lab.jar`.

All runtime dependencies (FlatLaf dark theme, JUnit 5) are declared in `pom.xml`
and downloaded automatically by Maven on first build. No manual dependency
installation is required.

---

## Running

```bash
# GUI mode (default) — run from the project root so config.properties is found
java -jar target/drone-lab.jar

# Headless mode — prints ranked comparison to stdout, no window
java -jar target/drone-lab.jar --headless
```

> **Important:** always run from the **project root directory** (the folder
> containing `pom.xml` and `config.properties`). The simulator looks for
> `config.properties` in the current working directory at startup.

---

## IDE Setup (IntelliJ IDEA)

1. **Open Project** → select the `drone-lab` folder (the one with `pom.xml`).
   IntelliJ detects Maven automatically.
2. Wait for the Maven import and indexing to finish.
3. Open `src/main/java/Main.java` and click the green **Run** button, or
   right-click → *Run 'Main'*.
4. To run headless from IntelliJ: **Edit Run Configuration** → add
   `--headless` to *Program arguments*.

To run tests: right-click `src/test/java` → *Run All Tests*, or run `mvn test`
in the integrated terminal.

---

## Configuration

All runtime parameters live in `config.properties` at the project root. The file
is loaded once at startup by `AppConfig`. Key settings:

```properties
run.headless      = false          # true = no GUI
run.seed          =                # integer for reproducible runs; empty = random
run.intelligences = patrol,gaaco   # algorithms to compare (comma-separated)

simulation.fieldWidth     = 1500
simulation.fieldHeight    = 1500
simulation.nodeCount      = 50
simulation.droneRange     = 140
simulation.droneSpeed     = 1
simulation.duration       = 5434
```

Full reference: see the [config.properties Reference](README.md#configproperties-reference)
section in `README.md`.

---

## Dependencies (auto-managed by Maven)

| Dependency | Version | Purpose |
|---|---|---|
| `com.formdev:flatlaf` | 3.x | Darcula/FlatLaf dark theme for the Swing GUI |
| `org.junit.jupiter:junit-jupiter` | 5.x | Unit testing framework |

These are declared in `pom.xml` and fetched from Maven Central on first build.
You do not need to install them manually.

---

## Troubleshooting

### `config.properties not found` or default parameters used
Run from the project root, not from `target/` or another directory:
```bash
cd /path/to/drone-lab
java -jar target/drone-lab.jar
```

### GUI does not appear / blank window
Ensure `run.headless = false` in `config.properties`. On Linux, verify a display
is available (`echo $DISPLAY`). Remote SSH sessions require X forwarding
(`ssh -X`) or use headless mode instead.

### `UnsupportedClassVersionError`
Your JRE is older than Java 17. Check with `java -version` and upgrade.

### Maven build fails on first run (download errors)
A proxy or firewall may be blocking Maven Central. Configure a proxy in
`~/.m2/settings.xml`, or download dependencies on a machine with internet
access and copy the `.m2` cache.

### Tests fail after editing config.properties
Some tests use `SimulationConfig.defaults()` which reads `config.properties`.
Check that the values you changed are still valid (positive integers, valid
ranges). Run `mvn test` to confirm all 265 tests pass after any change.
