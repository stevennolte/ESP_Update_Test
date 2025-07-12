import json

with open("firmware.json", "r") as f:
    data = json.load(f)

# Assume version is in the format "major.minor.patch"
version = data.get("version", "0.0.0")
major, minor, patch = map(int, version.split("."))

# Increment patch version
patch += 1
data["version"] = f"{major}.{minor}.{patch}"

with open("firmware.json", "w") as f:
    json.dump(data, f, indent=2)