# NKSwerve Build Scaffolding & Repo Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn `NKSwerve/` into a complete, standalone Gradle vendor-library project (matching `wpilibsuite/vendor-template`'s `2027` branch, C++-only) and reduce the repo root to just `archive/`, `NKSwerve/`, `README.md`, and `WPILib-License.md`.

**Architecture:** Copy the existing repo-root Gradle wrapper into `NKSwerve/` and repoint it at Gradle 9.4.1, then add `NKSwerve`'s own `build.gradle`/`config.gradle`/`publish.gradle`/`settings.gradle` (C++-only `NativeLibrarySpec` + GoogleTest suite, `org.wpilib`-groupId plugins, `systemcore` platform), verify the project configures (and that `compileCpp` fails only where expected), then delete everything else at the repo root and rewrite `README.md`.

**Tech Stack:** Gradle 9.4.1, `org.wpilib.*` GradleRIO-successor plugins (2027.x), C++, GoogleTest.

## Global Constraints

- No changes to `NKSwerve/src/main/native/{cpp,include}/*` in this plan — the `frc::`/`wpi::math::` API mismatch is deferred to a future spec (per [`2026-08-03-nkswerve-build-scaffolding-migration-design.md`](../specs/2026-08-03-nkswerve-build-scaffolding-migration-design.md)).
- C++-only: no `java` plugin, no `org.wpilib.GradleJni`, no driver library, no `symbols.txt`.
- Published artifact identity: `groupId = 'org.nasaknights'`, `baseArtifactId = 'NKSwerve'` (→ `NKSwerve-cpp`, `NKSwerve-vendordep`).
- Vendordep UUID: `8942ea4d-5e06-474b-b937-885212c33efe`.
- Success criterion for the Gradle project is **configuration succeeding**, not full compilation. `compileCpp` failing specifically on `SwerveDrive.cpp`/`SwerveModule.cpp` (the known `frc::`/`wpi::math::` mismatch) is an expected, acceptable outcome — not a bug to fix in this plan.
- `archive/` is never touched by any task in this plan.
- `.github/` is deleted outright, not preserved or updated (confirmed with the user — CI is not needed right now).

---

## Task 1: NKSwerve Gradle wrapper + settings.gradle

**Files:**
- Create: `NKSwerve/gradlew` (copied from repo-root `gradlew`)
- Create: `NKSwerve/gradlew.bat` (copied from repo-root `gradlew.bat`)
- Create: `NKSwerve/gradle/wrapper/gradle-wrapper.jar` (copied from repo-root `gradle/wrapper/gradle-wrapper.jar` — this bootstrap jar is not tied to a specific Gradle version, so it's safe to reuse rather than needing to fetch a new one)
- Create: `NKSwerve/gradle/wrapper/gradle-wrapper.properties`

**Interfaces:**
- Consumes: nothing.
- Produces: a runnable `./gradlew` inside `NKSwerve/`, and the `pluginManagement` repository config every later task's `build.gradle` plugin resolution depends on.

- [ ] **Step 1: Copy the wrapper launcher files from the repo root**

```bash
mkdir -p NKSwerve/gradle/wrapper
cp gradlew NKSwerve/gradlew
cp gradlew.bat NKSwerve/gradlew.bat
cp gradle/wrapper/gradle-wrapper.jar NKSwerve/gradle/wrapper/gradle-wrapper.jar
chmod +x NKSwerve/gradlew
```

- [ ] **Step 2: Write `NKSwerve/gradle/wrapper/gradle-wrapper.properties`**

```properties
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-9.4.1-bin.zip
networkTimeout=10000
validateDistributionUrl=true
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
```

- [ ] **Step 3: Write `NKSwerve/settings.gradle`**

```gradle
pluginManagement {
    repositories {
        mavenLocal()
        gradlePluginPortal()
    }
}
```

- [ ] **Step 4: Verify the wrapper runs**

Run: `cd NKSwerve && ./gradlew --version`
Expected: Gradle downloads (or resolves from cache) version `9.4.1` and prints its version banner — no `build.gradle` is needed for `--version` to work, so this succeeds even before Task 2. If this fails on a network error, note it and continue (Task 2's `./gradlew tasks` is the real gate); if it fails on anything else (e.g. a corrupt jar), stop and investigate before continuing.

- [ ] **Step 5: Commit**

```bash
git add NKSwerve/gradlew NKSwerve/gradlew.bat NKSwerve/gradle/wrapper/gradle-wrapper.jar NKSwerve/gradle/wrapper/gradle-wrapper.properties NKSwerve/settings.gradle
git commit -m "Add NKSwerve Gradle wrapper and settings.gradle"
```

---

## Task 2: NKSwerve build definition (build.gradle, config.gradle, publish.gradle, LICENSE.txt, NKSwerve.json)

**Files:**
- Create: `NKSwerve/build.gradle`
- Create: `NKSwerve/config.gradle`
- Create: `NKSwerve/publish.gradle`
- Create: `NKSwerve/LICENSE.txt`
- Create: `NKSwerve/NKSwerve.json`

**Interfaces:**
- Consumes: `NKSwerve/settings.gradle` and the wrapper from Task 1; `NKSwerve/src/main/native/{cpp,include}/*` (existing, untouched — this task only sources them, never edits them).
- Produces: a Gradle project that configures the `NKSwerve` (`NativeLibrarySpec`) component and `NKSwerveTest` (GoogleTest) suite, and can run `publishToMavenLocal` (Task 3 verifies this actually configures/builds).

- [ ] **Step 1: Write `NKSwerve/config.gradle`**

Carried over verbatim from `wpilibsuite/vendor-template`'s `2027` branch — generic WPILib native toolchain/cross-compile wiring, nothing vendor-specific:

```gradle
import org.gradle.internal.os.OperatingSystem

nativeUtils.addWpiNativeUtils()
nativeUtils.withCrossSystemCore()
nativeUtils.withCrossLinuxArm64()

nativeUtils {
    wpi {
        configureDependencies {
            wpiVersion = wpilibVersion
            mrcLibVersion = '2027.1.0-alpha-1-91-gb154ee9'
            opencvVersion = "2027-4.13.0-3"
        }
    }
}

nativeUtils.wpi.addWarnings()
nativeUtils.wpi.addWarningsAsErrors()

nativeUtils.setSinglePrintPerPlatform()

model {
    components {
        all {
            nativeUtils.useAllPlatforms(it)
        }
    }
    binaries {
        withType(NativeBinarySpec).all {
            nativeUtils.usePlatformArguments(it)
        }
    }
}

ext.appendDebugPathToBinaries = { binaries->
    binaries.withType(StaticLibraryBinarySpec) {
        if (it.buildType.name.contains('debug')) {
            def staticFileDir = it.staticLibraryFile.parentFile
            def staticFileName = it.staticLibraryFile.name
            def staticFileExtension = staticFileName.substring(staticFileName.lastIndexOf('.'))
            staticFileName = staticFileName.substring(0, staticFileName.lastIndexOf('.'))
            staticFileName = staticFileName + 'd' + staticFileExtension
            def newStaticFile = new File(staticFileDir, staticFileName)
            it.staticLibraryFile = newStaticFile
        }
    }
    binaries.withType(SharedLibraryBinarySpec) {
        if (it.buildType.name.contains('debug')) {
            def sharedFileDir = it.sharedLibraryFile.parentFile
            def sharedFileName = it.sharedLibraryFile.name
            def sharedFileExtension = sharedFileName.substring(sharedFileName.lastIndexOf('.'))
            sharedFileName = sharedFileName.substring(0, sharedFileName.lastIndexOf('.'))
            sharedFileName = sharedFileName + 'd' + sharedFileExtension
            def newSharedFile = new File(sharedFileDir, sharedFileName)

            def sharedLinkFileDir = it.sharedLibraryLinkFile.parentFile
            def sharedLinkFileName = it.sharedLibraryLinkFile.name
            def sharedLinkFileExtension = sharedLinkFileName.substring(sharedLinkFileName.lastIndexOf('.'))
            sharedLinkFileName = sharedLinkFileName.substring(0, sharedLinkFileName.lastIndexOf('.'))
            sharedLinkFileName = sharedLinkFileName + 'd' + sharedLinkFileExtension
            def newLinkFile = new File(sharedLinkFileDir, sharedLinkFileName)

            it.sharedLibraryLinkFile = newLinkFile
            it.sharedLibraryFile = newSharedFile
        }
    }
}

ext.createComponentZipTasks = { components, names, base, type, project, func ->
    def stringNames = names.collect {it.toString()}
    def configMap = [:]
    components.each {
        if (it in NativeLibrarySpec && stringNames.contains(it.name)) {
            it.binaries.each {
                if (!it.buildable) return
                def target = nativeUtils.getPublishClassifier(it)
                if (configMap.containsKey(target)) {
                    configMap.get(target).add(it)
                } else {
                    configMap.put(target, [])
                    configMap.get(target).add(it)
                }
            }
        }
    }
    def taskList = []
    def outputsFolder = file("$project.buildDir/outputs")
    configMap.each { key, value ->
        def task = project.tasks.create(base + "-${key}", type) {
            description = 'Creates component archive for platform ' + key
            destinationDirectory = outputsFolder
            archiveClassifier = key
            archiveBaseName = base
            duplicatesStrategy = 'exclude'

            from(licenseFile) {
                into '/'
            }

            func(it, value)
        }
        taskList.add(task)

        project.build.dependsOn task

        project.artifacts {
            task
        }
        addTaskToCopyAllOutputs(task)
    }
    return taskList
}

ext.includeStandardZipFormat = { task, value ->
    value.each { binary ->
        if (binary.buildable) {
            if (binary instanceof SharedLibraryBinarySpec) {
                task.dependsOn binary.tasks.link
                task.from(new File(binary.sharedLibraryFile.absolutePath + ".debug")) {
                    into nativeUtils.getPlatformPath(binary) + '/shared'
                }
                def sharedPath = binary.sharedLibraryFile.absolutePath
                sharedPath = sharedPath.substring(0, sharedPath.length() - 4)

                task.from(new File(sharedPath + '.pdb')) {
                    into nativeUtils.getPlatformPath(binary) + '/shared'
                }
                task.from(binary.sharedLibraryFile) {
                    into nativeUtils.getPlatformPath(binary) + '/shared'
                }
                task.from(binary.sharedLibraryLinkFile) {
                    into nativeUtils.getPlatformPath(binary) + '/shared'
                }
            } else  if (binary instanceof StaticLibraryBinarySpec) {
                task.dependsOn binary.tasks.createStaticLib
                task.from(binary.staticLibraryFile) {
                    into nativeUtils.getPlatformPath(binary) + '/static'
                }
            }
        }
    }
}
```

- [ ] **Step 2: Write `NKSwerve/build.gradle`**

```gradle
import org.wpilib.toolchain.*

plugins {
  id 'cpp'
  id 'google-test'
  id 'org.wpilib.WPILibRepositoriesPlugin' version '2027.0.0'
  id 'org.wpilib.NativeUtils' version '2027.13.1'
  id 'org.wpilib.GradleVsCode' version '2027.0.0'
}

ext.wpilibVersion = "2027.+"

repositories {
  mavenCentral()
}
wpilibRepositories.use2027Repos()
if (project.hasProperty('releaseMode')) {
  wpilibRepositories.addAllReleaseRepositories(project)
} else {
  wpilibRepositories.addAllDevelopmentRepositories(project)
}

apply from: 'config.gradle'

nativeUtils {
  exportsConfigs {
    NKSwerve {
    }
  }
}

model {
  components {
    NKSwerve(NativeLibrarySpec) {
      sources {
        cpp {
          source {
            srcDirs 'src/main/native/cpp'
            include '**/*.cpp'
          }
          exportedHeaders {
            srcDirs 'src/main/native/include'
          }
        }
      }
      nativeUtils.useRequiredLibrary(it, 'wpilib_shared')
    }
  }
  testSuites {
    NKSwerveTest {
        sources.cpp {
            source {
                srcDir 'src/test/native/cpp'
                include '**/*.cpp'
            }
        }
        nativeUtils.useRequiredLibrary(it, "wpilib_shared", "googletest_static")
    }
  }
}

apply from: 'publish.gradle'

wrapper {
  gradleVersion = '9.4.1'
}
```

- [ ] **Step 3: Write `NKSwerve/LICENSE.txt`**

Exact content of the repo-root `WPILib-License.md` (the two files intentionally carry the same text):

```
Copyright (c) 2009-2024 FIRST and other WPILib contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of FIRST, WPILib, nor the names of other WPILib
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

- [ ] **Step 4: Write `NKSwerve/NKSwerve.json`**

The vendordep template (`${version}`/`${groupId}`/`${artifactId}` are filled in by `publish.gradle`'s `vendordepJson` task, matching its `expand(version: ..., groupId: ..., artifactId: ...)` call):

```json
{
    "fileName": "NKSwerve.json",
    "name": "NKSwerve",
    "version": "${version}",
    "frcYear": "2027",
    "uuid": "8942ea4d-5e06-474b-b937-885212c33efe",
    "mavenUrls": [],
    "jsonUrl": "",
    "cppDependencies": [
        {
            "groupId": "${groupId}",
            "artifactId": "${artifactId}",
            "version": "${version}",
            "libName": "NKSwerve",
            "headerClassifier": "headers",
            "sharedLibrary": false,
            "skipInvalidPlatforms": true,
            "binaryPlatforms": [
                "linuxsystemcore",
                "linuxarm64",
                "windowsx86-64",
                "linuxx86-64",
                "osxuniversal"
            ]
        }
    ]
}
```

- [ ] **Step 5: Write `NKSwerve/publish.gradle`**

Adapted from `wpilibsuite/vendor-template`'s `2027` branch, trimmed to publish only the single native-library artifact (headers + sources + binaries) plus the vendordep JSON — no `java`/driver tasks or publications:

```gradle
import org.apache.tools.ant.filters.FixCrLfFilter
import org.apache.tools.ant.filters.ReplaceTokens

apply plugin: 'maven-publish'

ext.licenseFile = files("$rootDir/LICENSE.txt")

def templateVendorFile = "NKSwerve.json"

def pubVersion = '0.0.1'

def outputsFolder = file("$buildDir/outputs")

def versionFile = file("$outputsFolder/version.txt")

task outputVersions() {
    description = 'Prints the versions of wpilib to a file for use by the downstream packaging project'
    group = 'Build'
    outputs.files(versionFile)

    doFirst {
        buildDir.mkdir()
        outputsFolder.mkdir()
    }

    doLast {
        versionFile.write pubVersion
    }
}

task libraryBuild() {}

build.dependsOn outputVersions

task copyAllOutputs(type: Copy) {
    destinationDir = file("$buildDir/allOutputs")
    from versionFile
    dependsOn outputVersions
}

build.dependsOn copyAllOutputs
copyAllOutputs.dependsOn outputVersions

ext.addTaskToCopyAllOutputs = { task ->
    copyAllOutputs.dependsOn task
    copyAllOutputs.inputs.file task.archiveFile
    copyAllOutputs.from task.archiveFile
}

def artifactGroupId = 'org.nasaknights'
def baseArtifactId = 'NKSwerve'
def zipBaseName = "_GROUP_org_nasaknights_ID_${baseArtifactId}-cpp_CLS"

task cppHeadersZip(type: Zip) {
    destinationDirectory = outputsFolder
    archiveBaseName = zipBaseName
    archiveClassifier = "headers"

    from(licenseFile) {
        into '/'
    }

    from('src/main/native/include') {
        into '/'
    }
}

task cppSourceZip(type: Zip) {
    destinationDirectory = outputsFolder
    archiveBaseName = zipBaseName
    archiveClassifier = "sources"

    from(licenseFile) {
        into '/'
    }

    from('src/main/native/cpp') {
        into '/'
    }
}

build.dependsOn cppHeadersZip
addTaskToCopyAllOutputs(cppHeadersZip)
build.dependsOn cppSourceZip
addTaskToCopyAllOutputs(cppSourceZip)

// Apply template variables from the vendordep file.
// Replaces ${VARIABLE} with VARIABLE: value in expand()
task vendordepJson() {
    description = 'Builds the vendordep json file.'
    group = 'Build'
    outputs.file("$buildDir/repos/$templateVendorFile")

    copy {
        from templateVendorFile
        into "$buildDir/repos/"
        expand(version: pubVersion,
                groupId: artifactGroupId,
                artifactId: baseArtifactId)
    }
}

task vendordepJsonZip(type: Zip) {
    destinationDirectory = outputsFolder
    archiveBaseName = "vendordepJson"

    from("$buildDir/repos/$templateVendorFile") {
        into '/'
    }
    dependsOn vendordepJson
}

addTaskToCopyAllOutputs(vendordepJsonZip)
build.dependsOn vendordepJsonZip

libraryBuild.dependsOn build

def releasesRepoUrl = "$buildDir/repos/releases"

publishing {
    repositories {
        maven {
            url = releasesRepoUrl
        }
    }
}

task cleanReleaseRepo(type: Delete) {
    delete releasesRepoUrl
}

tasks.matching {it != cleanReleaseRepo}.all {it.dependsOn cleanReleaseRepo}

model {
    publishing {
        def taskList = createComponentZipTasks($.components, ['NKSwerve'], zipBaseName, Zip, project, includeStandardZipFormat)

        publications {
            cpp(MavenPublication) {
                taskList.each {
                    artifact it
                }
                artifact cppHeadersZip
                artifact cppSourceZip

                artifactId = "${baseArtifactId}-cpp"
                groupId = artifactGroupId
                version = pubVersion
            }

            vendordep(MavenPublication) {
                artifact vendordepJsonZip

                artifactId = "${baseArtifactId}-vendordep"
                groupId = artifactGroupId
                version = pubVersion
            }
        }
    }
}
```

- [ ] **Step 6: Verify the project configures**

Run: `cd NKSwerve && ./gradlew tasks`
Expected: the task list prints successfully (Gradle configuration phase completes with no errors) and includes standard `cpp`/native-library tasks. If configuration fails, read the error: a plugin-resolution failure (can't find `org.wpilib.*` plugins) most likely means `wpilibRepositories.use2027Repos()` isn't finding the local `C:\Users\Public\wpilib\2027_alpha5\maven` repo — check that install still exists before debugging further; any other configuration error should be fixed before moving on (this task's files must be internally consistent, unlike the deliberately-deferred `compileCpp` failure covered in Task 3).

- [ ] **Step 7: Commit**

```bash
git add NKSwerve/build.gradle NKSwerve/config.gradle NKSwerve/publish.gradle NKSwerve/LICENSE.txt NKSwerve/NKSwerve.json
git commit -m "Add NKSwerve build, config, and publish Gradle files"
```

---

## Task 3: Placeholder test suite + full build verification

**Files:**
- Create: `NKSwerve/src/test/native/cpp/PlaceholderTest.cpp`

**Interfaces:**
- Consumes: the `NKSwerveTest` test-suite definition from Task 2's `build.gradle`.
- Produces: proof that WPILib/GoogleTest linkage works end-to-end, independent of whether `NKSwerve`'s own `SwerveDrive.cpp`/`SwerveModule.cpp` compile.

- [ ] **Step 1: Write `NKSwerve/src/test/native/cpp/PlaceholderTest.cpp`**

Deliberately has zero dependency on any `NKSwerve` header — it only needs `wpilib_shared` + `googletest_static` (both precompiled WPILib-provided binaries), so it proves the test-suite wiring itself works even while `SwerveDrive.cpp`/`SwerveModule.cpp` (the main `NKSwerve` component's sources) are expected to fail to compile against 2027 alpha's real API:

```cpp
#include <gtest/gtest.h>

TEST(NKSwerveTest, Placeholder) {
    SUCCEED();
}
```

- [ ] **Step 2: Discover and run the `NKSwerveTest` execution task**

Run: `cd NKSwerve && ./gradlew tasks --all`
Native GoogleTest suites generate a platform/build-type-specific run task (exact name depends on the installed `NativeUtils` plugin version) — search the output for a task whose name contains `NKSwerveTest` and starts with `run`. Run that task, e.g.:

Run: `./gradlew <discovered task name>`
Expected: `PlaceholderTest.cpp` compiles, links, and the `NKSwerveTest.Placeholder` case passes. If this fails, the failure is in Task 1/2's scaffolding (wrapper, plugin wiring, or `useRequiredLibrary` linkage) — fix it, since `PlaceholderTest.cpp` has no dependency on the known-broken `NKSwerve` sources and is not allowed to fail per this plan.

- [ ] **Step 3: Confirm the expected, deferred `NKSwerve` component compile failure**

Run: `./gradlew build`
Expected: the build fails, and the failure is isolated to compiling `SwerveDrive.cpp` and/or `SwerveModule.cpp` (missing `frc/...h` headers, or symbols like `frc::ChassisSpeeds`/`frc::SwerveModuleState` not found — the documented `frc::` vs. `wpi::math::` mismatch from the design spec). If the failure is anywhere else (a `publish.gradle` task, `NKSwerveTest`, plugin configuration, or a missing file), that is a real bug introduced by this plan — stop and fix it. This is the concrete pass/fail check for the design spec's migration-plan step 7.

- [ ] **Step 4: Commit**

```bash
git add NKSwerve/src/test/native/cpp/PlaceholderTest.cpp
git commit -m "Add NKSwerve placeholder test suite"
```

---

## Task 4: Repo root cleanup

**Files:**
- Delete: `build.gradle`, `settings.gradle`, `gradle.properties`, `vendordeps/`, `.wpilib/`, `.vscode/`, `.Glass/`, `PathCalibrator/`, `networktables.json.bck`, `gradlew`, `gradlew.bat`, `gradle/`, `.github/` (all at the repo root — none of these are inside `NKSwerve/` or `archive/`)

**Interfaces:**
- Consumes: nothing new — Task 1 already copied everything `NKSwerve/` needs out of `gradlew`/`gradlew.bat`/`gradle/wrapper/gradle-wrapper.jar` before this task deletes the root copies.
- Produces: the target repo-root layout (`archive/`, `NKSwerve/`, `README.md`, `WPILib-License.md` only).

- [ ] **Step 1: Confirm nothing uncommitted would be lost**

Run: `git status`
Expected: no uncommitted changes inside any of the paths being deleted below (they were already established as safe to delete in the design spec — this step is a final check, not a decision point). If something unexpected shows up as uncommitted in one of these paths, stop and ask before deleting it.

- [ ] **Step 2: Delete the obsolete root files and directories**

```bash
git rm -r build.gradle settings.gradle gradle.properties vendordeps .wpilib .vscode .Glass PathCalibrator networktables.json.bck gradlew gradlew.bat gradle .github
```

- [ ] **Step 3: Verify the root layout**

Run: `ls -a`
Expected: only `archive/`, `NKSwerve/`, `README.md`, `WPILib-License.md`, `.git/`, `.gitignore`, `.gitmodules`, `docs/` remain at the top level (`.gitignore`/`.gitmodules`/`docs/` were never in the deletion list and are untouched).

- [ ] **Step 4: Re-verify `NKSwerve/` still stands alone**

Run: `cd NKSwerve && ./gradlew tasks`
Expected: still succeeds, exactly as in Task 2 Step 6 — this confirms `NKSwerve/` was never accidentally depending on anything at the old repo root.

- [ ] **Step 5: Commit**

```bash
git commit -m "Remove obsolete 2025 robot-deploy project files from repo root"
```

---

## Task 5: Rewrite README.md

**Files:**
- Modify: `README.md` (full rewrite)

**Interfaces:**
- Consumes: nothing.
- Produces: the repo's top-level documentation, pointing readers at `NKSwerve/` and the design spec.

- [ ] **Step 1: Read the current `README.md`**

Run: read the file to confirm what old competition-project content needs to be dropped (changelog entries, old robot-project setup instructions, etc.) before overwriting it.

- [ ] **Step 2: Write the new `README.md`**

```markdown
# NKSwerve

A C++ vendor library for FRC swerve drivetrain control, structured like
[`wpilibsuite/vendor-template`](https://github.com/wpilibsuite/vendor-template)'s
`2027` branch. Motors, steer actuators, and the gyro are all injected via
interfaces (`IDriveMotor`, `ISteerMotor`, `IGyroSource`) — `NKSwerve` itself
depends on WPILib only, never a specific hardware vendor.

See [`docs/superpowers/specs/2026-07-20-nkswerve-vendor-library-design.md`](docs/superpowers/specs/2026-07-20-nkswerve-vendor-library-design.md)
for the full library design, and
[`docs/superpowers/specs/2026-08-03-nkswerve-build-scaffolding-migration-design.md`](docs/superpowers/specs/2026-08-03-nkswerve-build-scaffolding-migration-design.md)
for how the project is structured and built.

## Building

```bash
cd NKSwerve
./gradlew build
```

## Installing in a robot project

```bash
cd NKSwerve
./gradlew publishToMavenLocal
```

Then, in a consuming robot project's `build.gradle`, add `mavenLocal()` to
`repositories`, and drop `NKSwerve/build/repos/NKSwerve.json` into that
project's `vendordeps/` directory.

## License

See [`WPILib-License.md`](WPILib-License.md).
```

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "Rewrite README for NKSwerve vendor library"
```

---

## Self-Review

**Spec coverage:**
- Gradle wrapper + `settings.gradle` — Task 1. ✓
- `build.gradle`, `config.gradle`, `publish.gradle`, `LICENSE.txt`, `NKSwerve.json` — Task 2. ✓
- Placeholder test suite + the spec's "configuration succeeds, compile fails only on the two known files" verification — Task 3. ✓
- Root cleanup (all twelve listed paths, `.github/` included per the user's explicit removal request) — Task 4. ✓
- `README.md` rewrite — Task 5. ✓
- Non-goals (API reconciliation, `ThreadFunction()` behavior, `testbed/`, remote publishing, real test coverage) — correctly not covered by any task. ✓

**Placeholder scan:** No "TBD"/"TODO" in any step. `PlaceholderTest.cpp`'s name and content are the spec's own explicit, deliberate scaffold (documented as such in both the spec and Task 3), not a gap.

**Type/name consistency:** Component name `NKSwerve` (Task 2's `build.gradle`) matches the `['NKSwerve']` list passed to `createComponentZipTasks` in `publish.gradle` (Task 2). Test suite name `NKSwerveTest` (Task 2's `build.gradle`) matches Task 3's task-discovery step. `artifactGroupId`/`baseArtifactId` (`org.nasaknights`/`NKSwerve`) are identical across `publish.gradle` and `NKSwerve.json`'s templated fields. `templateVendorFile = "NKSwerve.json"` (`publish.gradle`) matches the file created in Task 2 Step 4. `ext.licenseFile = files("$rootDir/LICENSE.txt")` resolves correctly since `NKSwerve/` is `rootDir` once it's its own Gradle project (Task 1 makes it standalone via its own `settings.gradle`).

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-03-nkswerve-build-scaffolding-migration-plan.md`. Two execution options:

1. **Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration
2. **Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
