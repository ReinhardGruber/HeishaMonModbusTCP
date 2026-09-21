"""PlatformIO post-build hook, also run when the firmware is already up to date."""

from pathlib import Path
import runpy

Import("env")


def copy_release(source, target, env):
    project = Path(env.subst("$PROJECT_DIR"))
    exporter = runpy.run_path(str(project / "scripts/export_firmware.py"))
    exporter["export_firmware"](
        project, Path(env.subst("$BUILD_DIR/${PROGNAME}.bin")), "esp32"
    )


env.AlwaysBuild(env.Alias("buildprog"))
env.AddPostAction("buildprog", copy_release)
