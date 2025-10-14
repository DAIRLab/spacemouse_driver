# spacemouse_driver
This repository provides a lightweight wrapper around `libspnav`, the client interface for `spacenavd`, to publish
SpaceMouse state data as LCM messages.

## Installation of Spacemouse Daemon (a low-level service that communicates directly with the hardware)

```sh
git clone https://github.com/FreeSpacenav/spacenavd
cd spacenavd
./configure
make
make install # You might need to run this command with sudo
```

To have the daemon start automatically on system boot, run:

```sh
sudo cp contrib/systemd/spacenavd.service /etc/systemd/system/
sudo systemctl start spacenavd

# To ensure the daemon is running, you can run
sudo systemctl status spacenavd
```

## Building LCM Driver for SpaceMouse devices

This package uses Bazel build system, you can install Bazel by following the [instruction](https://bazel.build/install/bazelisk).

Once Bazel is installed, you can build this driver as follows:

```sh
git clone https://github.com/DAIRLab/spacemouse_driver.git
cd spacemouse_driver
bazel build ... --keep_going
```

## Running LCM Driver

Under `spacemouse_driver` directory, simply run

```sh
bazel run //core:run_driver
```
