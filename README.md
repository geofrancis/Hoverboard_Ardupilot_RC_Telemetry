Hoverboard ESC manager,    
 control and telemetry via mavlink for multiple hoverboard controllers using a RP-2040.    

working:    
map left and right hoverboard channels to servo outputs 15 and 16.
send telemetry from up to 3 hoverboards to ardupilot as named floats. 

power management has been updated, it no longer uses the Arupilot Relay output to control the motors. 

when armed and in hold mode it will only start the centre ESC to hold position, once it changes to any other mode it will start the other 2 escs to drive.
automatic esc shut down on loss of mavlink heartbeat.

![hoverboard wiring](https://github.com/user-attachments/assets/d2a09282-b905-478c-9fef-c4fea551a423)

![Screenshot 2024-06-20 064302](https://github.com/geofrancis/Hoverboard_MAVLINK_RC_Telemetry/assets/5570278/07d68d38-3a74-4209-8ff4-57d675d888be)

![Screenshot 2024-06-20 065021](https://github.com/geofrancis/Hoverboard_MAVLINK_RC_Telemetry/assets/5570278/9b51a9fd-1ee1-4779-a814-7b1789b93073)
