# Arduino-HID-HOTAS-nrf52840-


hardware used:
nrf 52840 pro micro ,        i guess any nrf52840 will work and really arduino+ Adafruit_TinyUSB compatibility is the main necesitiy


SHKI ［10 Pack］ 608 2RS Ball Bearings – Bearing Steel and Double Rubber Sealed Miniature Deep Groove Ball Bearings for Skateboards, Inline Skates, Scooters (8mm x 22mm x 7mm) this is from amazon listing

(8mm x 22mm x 7mm) bearings



https://www.amazon.com/dp/B0F5QKDW7Y?ref=ppx_yo2ov_dt_b_fed_asin_title 
m3 screws 

mpu 9255   yes i used an IMU becuase i am that lazy, 

end of part list

wierd side note:
when 3d printed some parts are really weak , u will see in the fusion file there are screw bolts that aren't fastening multiple parts their only purpose is structural, no tolerences are given for those. it is tedious inserting them, but results in reinforced 3d printed.

imu why?   components of acceleration as in if gravity =  (0,0,1) when rotated components of g will give angle arctan is used , if u think about it a little its smart . if u think about it too much there are problems. 
if roll 90 degress then pitch , pitch is not measured because local rotation plus stuff i dont wanna do math, my code is kinda linear maybe..........
it was cheap and i had 1

will make this better 
