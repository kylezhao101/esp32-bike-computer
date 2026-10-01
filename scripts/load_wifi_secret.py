Import("env")

import json
import os
from pathlib import Path
from SCons.Script import Exit


project_dir = Path(env.subst("$PROJECT_DIR"))
secret_name = "BIKE_SETUP_PASSWORD"
values = {}
dotenv_path = project_dir / ".env"

if dotenv_path.is_file():
    for raw_line in dotenv_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        values[key] = value

password = os.environ.get(secret_name, values.get(secret_name, ""))
if password.startswith("replace-with-"):
    password = ""
if not 8 <= len(password) <= 63 or any(ord(ch) < 32 or ord(ch) > 126 for ch in password):
    print("ERROR: Set BIKE_SETUP_PASSWORD in the ignored .env file (8-63 printable ASCII characters).")
    Exit(1)

header_path = project_dir / "include" / "wifi_build_secrets.h"
header_path.write_text(
    "#pragma once\n"
    "#define BIKE_SETUP_PASSWORD " + json.dumps(password) + "\n",
    encoding="utf-8",
)
print("Loaded local Wi-Fi setup password for this build.")
