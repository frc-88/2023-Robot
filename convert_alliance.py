import json
import glob


JSON_PATH = "src\\main\\deploy\\pathplanner\\"
CONVERT_FROM_RED = False
FIELD_WIDTH = 16.54
Y_ADJUST = 0
X_ADJUST = 0

files_to_convert = ["CenterEngage.path"]
#Uncomment to do all files
files_to_convert = glob.glob(JSON_PATH + ("Red" if CONVERT_FROM_RED else "Blue") + "*.path")

for file_to_convert in files_to_convert:
    with open(file_to_convert) as f:
        data = json.load(f)
        
    # for waypoint in data["waypoints"]:
    #     if waypoint["anchorPoint"] is not None:
    #         waypoint["anchorPoint"]["x"] = waypoint["anchorPoint"]["x"] + X_ADJUST
    #         waypoint["anchorPoint"]["y"] = waypoint["anchorPoint"]["y"] + Y_ADJUST
    #     if waypoint["nextControl"] is not None:
    #         waypoint["nextPoint"]["x"] = waypoint["nextPoint"]["x"] + X_ADJUST
    #         waypoint["nextPoint"]["y"] = waypoint["nextPoint"]["y"] + Y_ADJUST
    #     if waypoint["prevControl"] is not None:
    #         waypoint["prevPoint"]["x"] = waypoint["prevPoint"]["x"] + X_ADJUST
    #         waypoint["prevPoint"]["y"] = waypoint["prevPoint"]["y"] + Y_ADJUST
            
    # with open(file_to_convert, 'w') as f:
    #     json.dump(data, f, indent=4)
        
    for waypoint in data["waypoints"]:
        if waypoint["anchorPoint"] is not None:
            waypoint["anchorPoint"]["x"] = FIELD_WIDTH - waypoint["anchorPoint"]["x"]
        if waypoint["nextControl"] is not None:
            waypoint["nextControl"]["x"] = FIELD_WIDTH - waypoint["nextControl"]["x"]
        if waypoint["prevControl"] is not None:
            waypoint["prevControl"]["x"] = FIELD_WIDTH - waypoint["prevControl"]["x"]
            
    with open(file_to_convert.replace("Red" if CONVERT_FROM_RED else "Blue", "Blue" if CONVERT_FROM_RED else "Red"), 'w') as f:
        json.dump(data, f, indent=4)
        
    


