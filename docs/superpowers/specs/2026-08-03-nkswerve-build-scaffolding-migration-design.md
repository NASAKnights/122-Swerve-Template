# NKSwerve Build Scaffolding & Repo Migration — Design Spec

## Context

Phase 1 ([`2026-07-20-nkswerve-vendor-library-design.md`](2026-07-20-nkswerve-vendor-library-design.md)) designed `NKSwerve` as a `wpilibsuite/vendor-template`-style C++ vendor library, and its "Migration plan" laid out four steps: (1) extract the C++ skeleton from `archive/`, (2) delete everything else at the repo root, (3) rewrite `README.md`, (4) verify the build.

The [2026-08-01 skeleton plan](../plans/2026-08-01-nkswerve-cpp-skeleton.md) executed step 1 only — `NKSwerve/src/main/native/{cpp,include}/` now has the interfaces, `SwerveModule`, and `SwerveDrive` structure — and explicitly deferred steps 2–4: *"Gradle/build scaffolding, root-directory cleanup, README rewrite, build verification — explicitly out of scope for this plan."* That plan also flagged an unresolved risk: the locally installed WPILib matching phase 1's declared `wpilibVersion = "2027.+"` might use a renamed C++ API.

This spec covers migration steps 2–4, and resolves the open questions phase 1 left about what "2027" actually means on disk:

- **WPILib groupId is `org.wpilib`, not `edu.wpi.first`.** Confirmed by inspecting `C:\Users\Public\wpilib\2027_alpha5\maven` — there is no `edu\wpi\first` tree at all in that install; everything (including `GradleRIO`, `wpimath`, etc.) lives under `org\wpilib\`.
- **The RoboRIO target platform is renamed `systemcore`.** Vendor-template artifacts publish as `-linuxsystemcore.zip`, not `-linuxathena.zip`.
- **The C++ API itself is renamed and restructured**, confirmed by extracting `wpimath-cpp-2027.0.0-alpha-6-headers.zip`:
  - `frc::` → `wpi::math::`, headers move from `frc/geometry/Rotation2d.h` to `wpi/math/geometry/Rotation2d.hpp`.
  - `frc::ChassisSpeeds` no longer exists — split into `wpi::math::ChassisVelocities` and `wpi::math::ChassisAccelerations`.
  - `frc::SwerveModuleState` no longer exists — split into `wpi::math::SwerveModuleVelocity` and `wpi::math::SwerveModuleAcceleration`.
  - `wpi::array<T, N>` → `wpi::util::array<T, N>`.

**Decision for this spec (confirmed with the user):** reconciling `NKSwerve`'s C++ source against this renamed API is deferred to a future spec. This spec's job is only to get the *project structure* — the Gradle scaffolding and repo layout — into its target shape. It is expected and acceptable that `compileCpp` fails once the build reaches actual compilation of `SwerveDrive.cpp`/`SwerveModule.cpp` against the real 2027 alpha headers. Success here means the Gradle project *configures* cleanly (plugins resolve, the component/test-suite model builds), not that it compiles.

## Goals

- Give `NKSwerve/` a complete, standalone Gradle project structurally matching [`wpilibsuite/vendor-template`](https://github.com/wpilibsuite/vendor-template)'s `2027` branch, trimmed to C++-only (no `java` plugin, no JNI driver split — confirmed decision, since `NKSwerve` has no hardware I/O of its own to bridge to Java).
- Reflect the verified 2027-alpha-6 specifics above: `org.wpilib` groupId, `systemcore` platform, the split plugin set (`org.wpilib.WPILibRepositoriesPlugin`, `org.wpilib.NativeUtils`, `org.wpilib.GradleVsCode` — no `org.wpilib.GradleJni`), Gradle wrapper `9.4.1`.
- Reduce the repo root to just `archive/` (untouched), `NKSwerve/` (fully restructured), `README.md` (rewritten), and `WPILib-License.md` (kept as-is) — removing the 2025 robot-deploy project entirely.
- Leave `NKSwerve/` in a state where `./gradlew tasks` (or equivalent project-configuration step) succeeds, even though `./gradlew build`/`compileCpp` is expected to fail on the API mismatch.

## Non-goals

- Reconciling `NKSwerve`'s C++ source with 2027 alpha's actual `wpi::math::` API (the `ChassisVelocities`/`ChassisAccelerations` and `SwerveModuleVelocity`/`SwerveModuleAcceleration` splits in particular need a real design decision, not a mechanical rename) — deferred to a future spec.
- `SwerveDrive::ThreadFunction()` functional behavior — already deferred by the skeleton plan, still deferred here.
- The `testbed/` consuming robot project — separate existing spec ([`2026-07-20-nkswerve-testbed-robot-design.md`](2026-07-20-nkswerve-testbed-robot-design.md)), untouched by this spec.
- Publishing to any real remote Maven host — `publishToMavenLocal` only, per phase 1.
- CI: `.github/workflows/build.yml` is **deleted** as part of this spec's root cleanup (confirmed with the user — not needed right now), not merely left stale. Re-adding CI for `NKSwerve/` is a future fast-follow if wanted.
- Real unit test coverage for `SwerveModule`/`SwerveDrive` logic — deferred alongside the API reconciliation, since writing tests against types that don't compile isn't useful yet. This spec only wires up an empty, structurally-valid test suite.

## Target repo layout

```
Swerve_Drive_Template/
├── archive/                     (untouched)
├── NKSwerve/
│   ├── build.gradle             (new)
│   ├── config.gradle            (new)
│   ├── publish.gradle           (new)
│   ├── settings.gradle          (new)
│   ├── gradlew / gradlew.bat / gradle/wrapper/gradle-wrapper.properties   (new, Gradle 9.4.1)
│   ├── LICENSE.txt              (new — content carried over from root WPILib-License.md)
│   ├── NKSwerve.json            (new — vendordep JSON template)
│   ├── src/main/native/cpp/
│   │   ├── SwerveDrive.cpp      (existing, moved as-is — no source changes in this spec)
│   │   └── SwerveModule.cpp     (existing, moved as-is)
│   ├── src/main/native/include/
│   │   ├── SwerveDrive.h, SwerveModule.h, SwerveDriveConfig.h
│   │   └── IGyroSource.h, IDriveMotor.h, ISteerMotor.h, IMutex.h    (existing, moved as-is)
│   └── src/test/native/cpp/
│       └── PlaceholderTest.cpp   (new — one trivial passing test, proves suite wiring only)
├── README.md                    (rewritten — describes NKSwerve as a vendor library, drops old competition-project content)
└── WPILib-License.md            (untouched)
```

Everything else currently at the repo root is deleted as part of this spec: `build.gradle`, `settings.gradle`, `gradle.properties`, `vendordeps/`, `.wpilib/`, `.vscode/`, `.Glass/`, `PathCalibrator/`, `networktables.json.bck`, root `gradlew`/`gradlew.bat`/`gradle/`, `.github/`. `archive/` is explicitly excluded from deletion — it stays exactly as it is.

## `NKSwerve/build.gradle`

Adapted from vendor-template's `2027` branch, C++-only (no `java`, no `org.wpilib.GradleJni`, no `VendorDriver` component, no `privateExportsConfigs`):

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

## `NKSwerve/config.gradle`

Carried over near-verbatim from the upstream template — generic WPILib toolchain/cross-compile wiring (`systemcore`, `linuxarm64`, debug-binary suffixing). Nothing vendor-specific to change; this file has no Java/driver-specific content in the upstream template to begin with, so no trimming is needed beyond copying it in as-is.

## `NKSwerve/publish.gradle`

Adapted from the template, trimmed to publish only the single native-library artifact (headers + sources + binaries) plus the vendordep JSON — no `java`/`sourcesJar`/`javadocJar`/`outputJar` tasks (those depend on the `java` plugin's `classes`/`javadoc` tasks, which aren't applied), no driver headers zip, no driver `MavenPublication`:

- Keep: `outputVersions`, `libraryBuild`, `copyAllOutputs`, `cppHeadersZip`, `cppSourceZip`, `vendordepJson`, `vendordepJsonZip`, the `cpp` publication, the `vendordep` publication.
- Drop: `cppDriverHeadersZip`, `sourcesJar`, `javadocJar`, `outputJar`, `outputSourcesJar`, `outputJavadocJar`, the `driver` publication, the `java` publication, and `VendorDriver` from the `createComponentZipTasks` call (only `['NKSwerve']` remains, matching the renamed component).
- `artifactGroupId = 'org.nasaknights'`, `baseArtifactId = 'NKSwerve'` (per phase 1's decision) — artifacts publish as `NKSwerve-cpp` and `NKSwerve-vendordep`.
- `ext.licenseFile = files("$rootDir/LICENSE.txt")` stays as in the template — resolves to `NKSwerve/LICENSE.txt` since `rootDir` is `NKSwerve/` once it's its own Gradle project.
- `templateVendorFile = "NKSwerve.json"` (renamed from `ExampleVendorJson.json`).

## `NKSwerve/settings.gradle`

Simplified — no hand-rolled `frcHome` pointer (that mechanism is obsolete; `wpilibRepositories.use2027Repos()` in `build.gradle` handles repo resolution now):

```gradle
pluginManagement {
    repositories {
        mavenLocal()
        gradlePluginPortal()
    }
}
```

## `NKSwerve/NKSwerve.json` (vendordep template)

Single `cppDependencies` entry, `libName: "NKSwerve"`, `groupId: org.nasaknights`, `artifactId: NKSwerve-cpp`, templated `${version}` (filled in by `publish.gradle`'s `vendordepJson` task at build time). No `javaDependencies`, no `jniDependencies`. UUID: `8942ea4d-5e06-474b-b937-885212c33efe` (freshly generated for this library).

## `NKSwerve/LICENSE.txt`

Exact content of the current root `WPILib-License.md` (the standard WPILib BSD-3-Clause-style license), copied in as required by `publish.gradle`'s `licenseFile` reference. Root `WPILib-License.md` is untouched — the two files intentionally carry the same text for now.

## `src/test/native/cpp/PlaceholderTest.cpp`

One trivial GoogleTest case (e.g. `TEST(NKSwerveTest, Placeholder) { SUCCEED(); }`) with no dependency on `SwerveDrive`/`SwerveModule` — proves the `NKSwerveTest` suite is wired correctly without needing the API-mismatch problem solved first. Real coverage is added in the future API-reconciliation spec.

## `README.md`

Rewritten to describe `NKSwerve` as a C++ vendor library for swerve drivetrain control (structure, install instructions via `publishToMavenLocal`, link to the phase 1 design spec), dropping any old competition-robot-project content.

## Migration plan (executed during implementation, not during spec-writing)

1. Create `NKSwerve/build.gradle`, `config.gradle`, `publish.gradle`, `settings.gradle` as specified above.
2. Add `NKSwerve/gradlew`, `gradlew.bat`, `gradle/wrapper/gradle-wrapper.properties` (Gradle `9.4.1`, matching the vendor-template's wrapper).
3. Add `NKSwerve/LICENSE.txt` (copied from root `WPILib-License.md`) and `NKSwerve/NKSwerve.json` (vendordep template, UUID above).
4. Add `NKSwerve/src/test/native/cpp/PlaceholderTest.cpp`.
5. Delete from the repo root: `build.gradle`, `settings.gradle`, `gradle.properties`, `vendordeps/`, `.wpilib/`, `.vscode/`, `.Glass/`, `PathCalibrator/`, `networktables.json.bck`, `gradlew`, `gradlew.bat`, `gradle/`, `.github/`. Leave `archive/`, `NKSwerve/`, `WPILib-License.md` untouched.
6. Rewrite `README.md`.
7. From inside `NKSwerve/`, run a Gradle project-configuration step (e.g. `./gradlew tasks` or `./gradlew help`) and confirm it succeeds — plugins resolve, the `NKSwerve`/`NKSwerveTest` model builds. Then run `./gradlew build` and confirm it fails specifically at C++ compilation of `SwerveDrive.cpp`/`SwerveModule.cpp` (the known, expected, deferred API mismatch) rather than at project configuration or at any other file. This distinction is the actual verification step for this spec — configuration success plus a compile failure isolated to the two known files.

## Open items for the future API-reconciliation spec

- Whether `NKSwerve`'s public API (`SwerveDrive::Drive()`, `SwerveModule::SetDesiredState()`, etc.) should ignore `wpi::math::ChassisAccelerations`/`SwerveModuleAcceleration` entirely (closest to today's velocity-only behavior) or thread acceleration feedforward through as part of the migration — flagged during this spec's brainstorming, explicitly deferred.
- Mechanical rename of `frc::` → `wpi::math::`, `units::` → presumably `wpi::units::` (needs confirming against the installed headers the same way `wpi::math::` was confirmed), `wpi::array` → `wpi::util::array`, and header paths (`frc/geometry/*.h` → `wpi/math/geometry/*.hpp`) across all seven `NKSwerve/src/main/native/include/*.h` files and both `.cpp` files.
- Re-adding real GoogleTest coverage for `SwerveModule`/`SwerveDrive` logic once the above compiles.
- Re-adding CI (`.github/workflows/`) for `NKSwerve/`, if wanted — explicitly deferred by this spec.
