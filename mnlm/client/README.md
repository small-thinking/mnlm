# Client side of the voice control system for the robot arm.

Note: You need to run this client side outside of the docker, otherwise the voice control system will not work.

## Overview
This is the client side of the voice control system for the robot arm.
The users can control the robot arm using voice.

## Startup

To start the voice control system, run the following command in the folder mnlm/client/:
```
python gpt_control/assistant.py
```

## Usage
A number of configurations can be set. You can change the settings in the main function
in gpt_control/assistant.py to customize the voice control system.

```
verbose: bool
    If True, the system will print the recognized speech and the response.
    If False, the system will not print the recognized speech and the response.

nudge_user: bool
    If True, the system will nudge the user to speak if the user is silent for a long time.
    If False, the system will not nudge the user to speak.

use_voice_input: bool
    If True, the system will use voice input.
    If False, the system will use text input.

use_voice_output: bool
    If True, the system will use voice output.
    If False, the system will use text output.

use_dummy_robot_arm_server: bool
    If True, the system will use a dummy robot arm server that just print out messages.
    If False, the system will use the real robot arm server.
```