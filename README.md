# Teensy-Wakame
GPS Unit: Teensy 3.2, Ublox NEO6, LCD, HMC5883

This is a project to try and build a handheld GPS device.
Goals:
Use GPS to obtain
- latitude
- longitude
- altitude
- SOG
- bearing
- heading

Display results on an LCD
- Satellite map

Store waypoints and tracks

Current status:
- Parse and display GPS NMEA data
- Create 2D satellite map (refresh is buggy)
- Display bearing

Background:
Original development started on an ESP8266, but due to I/O constraints the Teensy 3.2 was chosen.
The cost of parts so far is around $50, but that can be reduced after prototyping with discrete components.

Video demo:
Here is a short video demonstratng the satellite map with acquired satellites. No fix had yet been acquired.
https://www.youtube.com/edit?o=U&feature=vm&video_id=ZwM4uvmy9Ao
