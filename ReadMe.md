# Scenario designer

Change from manual_control.py provided by carla. Press key "H" for help
## Main features:
1. Controlling only the camera, not driving a car which makes navigation easier
2. Provide two camera, one for local environment set up and exploration, another provides global perspective of the carla world
3. Placing and deleting several types of obstacles.
4. Storing obstacles in a single config file with the format needed by xml parser.

## Structure:

1. Class World          : manages other components, provide restart function to do initialization.
2. Class KeyboardControl: manages keyboard and mouse behavior in parse_events.
3. Class Hud            : manages the information provided on the screen.
4. Class CameraManager  : manages cameras and their movements.
5. Class ObjectManager  : manages the object we added and responsible for storing them in config file

## TODO:
1. Setup waypoints for control simulations.
2. Specify town name in the stored config file.