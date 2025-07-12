import json

with open("firmware.json", "r") as f:
    data = json.load(f)

# Assume version is an integer
version = int(data.get("version", 0))
version += 1
data["version"] = version

with open("firmware.json", "w") as f:
    json.dump(data, f, indent=2)