Import("env")
import os
import subprocess

framework_dir = env.PioPlatform().get_package_dir("framework-espidf")
requirements_file = os.path.join(
    framework_dir, "tools", "requirements", "requirements.core.txt"
)

pio_home = os.path.expanduser("~/.platformio")
penv_dir = os.path.join(pio_home, "penv")

espidf_venvs = sorted(
    [d for d in os.listdir(penv_dir) if d.startswith(".espidf-")]
)
if not espidf_venvs:
    print("WARNING: Could not find ESP-IDF venv under ~/.platformio/penv/")
else:
    espidf_python = os.path.join(penv_dir, espidf_venvs[-1], "bin", "python")
    # Only install if kconfgen is missing (avoids slow pip run on every build)
    probe = subprocess.run(
        [espidf_python, "-c", "import kconfgen, idf_component_manager"],
        capture_output=True,
    )
    if probe.returncode != 0:
        print("First-time setup: installing ESP-IDF Python requirements...")
        subprocess.check_call(
            [espidf_python, "-m", "pip", "install", "-q", "-r", requirements_file]
        )
        print("ESP-IDF requirements installed.")
