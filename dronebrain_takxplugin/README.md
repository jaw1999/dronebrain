# Rivet TAKX Plugin

## Description
Rivet TAKX plugin for integration with TAK systems.

## Download

### Latest Release
Download the latest version from GitLab Releases:

**Release Page:** https://code.levelup.cce.af.mil/mdeb/forge/rivet/-/releases

**Direct Download (requires GitLab login):**
```bash
curl -k --header "PRIVATE-TOKEN: YOUR_GITLAB_TOKEN" \
  "https://code.levelup.cce.af.mil/api/v4/projects/41290/packages/generic/rivet-takx-plugin/1.0.0/rivet-1.0.0-SNAPSHOT.jar" \
  -o rivet-1.0.0-SNAPSHOT.jar
```

### Maven/Gradle Dependency

Add to your `build.gradle`:
```gradle
repositories {
    maven {
        url = "https://code.levelup.cce.af.mil/api/v4/projects/41290/packages/maven"
        name = "GitLab"
        credentials(HttpHeaderCredentials) {
            name = "Private-Token"
            value = project.findProperty("gitlabToken") ?: System.getenv("GITLAB_TOKEN")
        }
        authentication {
            header(HttpHeaderAuthentication)
        }
    }
}

dependencies {
    implementation 'com.rivet:rivet:1.0.0-SNAPSHOT'
}
```

## Requirements
- Java 17
- TAKX 5.4.0 or later
- Linux operating system (configured for `/opt/TAKX`)

## Installation

1. Download the JAR file from the release page or using the command above
2. Copy to your TAKX plugins directory:
   ```bash
   cp rivet-1.0.0-SNAPSHOT.jar /opt/TAKX/plugins/
   ```
3. Restart TAKX

## Build from Source

### Prerequisites
- Java 17 JDK
- TAKX installation at `/opt/TAKX`
- Gradle wrapper (included)

### Build Command
```bash
JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 ./gradlew clean build --no-daemon --init-script disable-ssl.gradle
```

The built JAR will be located at:
```
build/libs/rivet-1.0.0-SNAPSHOT.jar
```

### Install to Local TAKX
```bash
JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 ./gradlew install
```

This will copy the JAR directly to `/opt/TAKX/plugins/`.

### Uninstall from Local TAKX
```bash
./gradlew uninstall
```

## Configuration

The plugin uses the following configuration in `build.gradle`:
- **Group:** `com.rivet`
- **Artifact:** `rivet`
- **Version:** `1.0.0-SNAPSHOT`
- **TAKX Version:** 5.4.0
- **Platform:** Linux (linux64)

To change the TAKX installation directory, update the `takxInstallDir` variable in `build.gradle`.

## Publishing New Releases

### Publish to GitLab Package Registry
```bash
JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 ./gradlew publish --no-daemon --init-script disable-ssl.gradle
```

This will publish to:
- Maven Package Registry: `https://code.levelup.cce.af.mil/api/v4/projects/41290/packages/maven`

### Create a GitLab Release

1. Build the JAR:
   ```bash
   JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 ./gradlew clean build
   ```

2. Tag the release:
   ```bash
   git tag -a v1.0.0 -m "Release v1.0.0"
   git push origin v1.0.0
   ```

3. Upload to Generic Package Registry:
   ```bash
   curl -k --header "PRIVATE-TOKEN: YOUR_TOKEN" \
     --upload-file build/libs/rivet-1.0.0-SNAPSHOT.jar \
     "https://code.levelup.cce.af.mil/api/v4/projects/41290/packages/generic/rivet-takx-plugin/1.0.0/rivet-1.0.0-SNAPSHOT.jar"
   ```

4. Create the release in GitLab UI or via API

## Development

### Project Structure
```
rivet_takxplugin/
├── build.gradle           # Build configuration
├── settings.gradle        # Project settings
├── gradle.properties      # Gradle properties
├── disable-ssl.gradle     # SSL bypass for GitLab
├── set-credentials.gradle # Credentials plugin
└── src/
    └── main/
        └── java/
            └── com/
                └── rivet/  # Plugin source code
```

### Dependencies
- TAKX Platform SDK: 5.4.0
- Jakarta Annotations API: 2.1.1
- Jakarta Enterprise CDI API: 4.0.1
- SLF4J API: 2.0.3
- TAK Kernel Shared: 8.18.1
- TAK Kernel Engine: 8.18.1
- NASA WorldWind: 2.2.1-TAKX-4

### Gradle Tasks
- `./gradlew build` - Build the plugin
- `./gradlew clean` - Clean build artifacts
- `./gradlew install` - Install to local TAKX
- `./gradlew uninstall` - Uninstall from local TAKX
- `./gradlew publish` - Publish to GitLab Package Registry

## Troubleshooting

### SSL Certificate Errors
If you encounter SSL certificate errors when publishing:
```bash
# Use the disable-ssl.gradle init script
./gradlew publish --init-script disable-ssl.gradle
```

### Java Version Issues
Ensure Java 17 is being used:
```bash
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
./gradlew --version
```

### TAKX Not Found
If the install task fails, verify your TAKX installation:
```bash
ls -la /opt/TAKX/plugins/
```

Update `takxInstallDir` in `build.gradle` if TAKX is installed elsewhere.


