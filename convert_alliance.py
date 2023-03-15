import json
import glob


JSON_PATH = "src/main/deploy/pathplanner/"
CONVERT_FROM_RED = True
FIELD_WIDTH = 16.54

files_to_convert = ["CenterEngage.path"]
#Uncomment to do all files
files_to_convert = glob.glob(JSON_PATH + ("Red" if CONVERT_FROM_RED else "Blue") + "*.path")

for file_to_convert in files_to_convert:
    with open(file_to_convert) as f:
        data = json.load(f)
        
    for waypoint in data["waypoints"]:
        if waypoint["anchorPoint"] is not None:
            waypoint["anchorPoint"]["x"] = FIELD_WIDTH - waypoint["anchorPoint"]["x"]
        if waypoint["nextControl"] is not None:
            waypoint["nextControl"]["x"] = FIELD_WIDTH - waypoint["nextControl"]["x"]
        if waypoint["prevControl"] is not None:
            waypoint["prevControl"]["x"] = FIELD_WIDTH - waypoint["prevControl"]["x"]
            
    with open(JSON_PATH + ("Blue" if CONVERT_FROM_RED else "Red") + file_to_convert, 'w') as f:
        json.dump(data, f, indent=4)
        
    


