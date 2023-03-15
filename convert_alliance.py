import json
import os

JSON_PATH = "src/main/deploy/pathplanner/"
FILE_TO_CONVERT = "CenterEngage.path"
CONVERT_FROM_RED = True
FIELD_WIDTH = 16.54


with open(JSON_PATH + ("Red" if CONVERT_FROM_RED else "Blue") + FILE_TO_CONVERT) as f:
    data = json.load(f)
    
for waypoint in data["waypoints"]:
    if waypoint["anchorPoint"] is not None:
        waypoint["anchorPoint"]["x"] = FIELD_WIDTH - waypoint["anchorPoint"]["x"]
    if waypoint["nextControl"] is not None:
        waypoint["nextControl"]["x"] = FIELD_WIDTH - waypoint["nextControl"]["x"]
    if waypoint["prevControl"] is not None:
        waypoint["prevControl"]["x"] = FIELD_WIDTH - waypoint["prevControl"]["x"]
        
with open(JSON_PATH + ("Blue" if CONVERT_FROM_RED else "Red") + FILE_TO_CONVERT, 'w') as f:
    json.dump(data, f, indent=4)
        
    


