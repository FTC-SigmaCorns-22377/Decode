# Setup

> **Prerequisites:** Make sure `git-lfs` and `cmake` are installed on your system before proceeding.

## 1. Create `local.properties`

Create a `local.properties` file in the root of the project with the following contents:

```properties
sdk.dir=/path/to/your/Android/sdk
username=<your GitHub username>
apiKey=<your GitHub personal access token>
```

- **`sdk.dir`** — path to your Android SDK. Android Studio usually sets this automatically.
- **`username`** — your GitHub username.
- **`apiKey`** — a GitHub personal access token with at least `read:packages` scope.

To create a personal access token:
1. Go to [github.com/settings/tokens](https://github.com/settings/tokens)
2. Click **Generate new token (classic)**
3. Give it a name and enable the **`read:packages`** scope
4. Click **Generate token** and copy the value into `local.properties`

> `local.properties` is gitignored and should never be committed.

## 2. Install the Android NDK

This project requires NDK version **`29.0.14033849`**.

To install it in Android Studio:
1. Open **Android Studio**
2. Go to **Settings** → **Languages & Frameworks** → **Android SDK** (or **SDK Manager** from the toolbar)
3. Click the **SDK Tools** tab
4. Check **Show Package Details** in the bottom right
5. Expand **NDK (Side by side)** and check **29.0.14033849**
6. Click **Apply** and let it download

## 3. Run the setup task

From the project root, run:

```bash
./gradlew setupDeps
```

This will:
- Initialize and update all git submodules (including nested ones)
- Check out the default branch in each submodule
- Pull Git LFS objects
- Set up the Python virtual environment for trajopt and install its dependencies

## Note: Rebuilding the native MPC library

If you make changes to `deps/mecanum_LTV_OCP`, you need to recompile the native library and copy it into `jniLibs` by running:

```bash
./gradlew :TeamCode:buildMecanumLtvOcp
```

This is also run automatically when deploying to the robot (`installDebug`).
