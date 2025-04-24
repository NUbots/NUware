Back-Panel IO
=============

## Brief
Displays information through a series of LEDs including two RGB LEDs and takes inputs in from the user through three buttons; one of which shuts of power to the servos in case of an emergency. As a further convenience, there are two headers to power two fans with 12 V. The PCB is connected to the subcontroller through two cables.

## Motivation
The back-panel IO [originally designed by Robotis](https://github.com/ROBOTIS-GIT/ROBOTIS-OP-Series-Data/blob/master/ROBOTIS-OP%2C%20ROBOTIS-OP2/Hardware/Electronics/Boards/DARwIn-OP_Interface_rev3.pdf) was modified slightly for the NUgus to have 3D-printed buttons. These buttons floated between the PCB and the outer panel. As such, they were very finicky and sometimes got themselves stuck. Furthermore, given that the underlying electrical switch was a generic tactile momentary switch with a very short actuation depth, there was little tactile feedback. That is to say that there wasn't a very satsifying click which sometimes made the user doubt whether a contact was made. (This would compound separate but related issues with the reliability of the button firmware.) Another grievance was the fact that the mounting holes of the PCB were near the pads of the switches; if a metal washer was used, then it would short the switch.

## Resistors
The original design by Robotis had only 470 Ω resistors for all LEDs including the RGB ones. This seemed inconsistent; so, 240 Ω resistors were used for the green and blue LEDs since they often have higher forward voltages. Going off from [the datasheet](https://s3-us-west-2.amazonaws.com/catsy.557/Dialight_CBI_data_598-0603_Apr2018.pdf) for the Dialight 598 series, these resistors should give a current of 2.5 mA in each LED and roughly a forward voltage of 2 V for the red and yellow-green LEDs, 2.6 V for the blue LED, and 2.75 V for the green LED.

## Switch
[The KS11R2xCQD](https://www.ckswitches.com/media/1342/ks.pdf) was chosen as the replacement switch since it had a longer travel of 0.71 mm, had shorter profile than similar alternatives, and was relatively cheaper.

An alternative for future revisions is [the 5501M series](https://sten-eswitch-13110800-production.s3.amazonaws.com/system/asset/product_line/data_sheet/24/5500.pdf) which has a longer travel of about 1 mm -- the datasheet is somewhat ambiguous. It can even have an embedded LED which can be wired to the second arm of a SPDT -- labelled NC in the datasheet -- and can be used to signify when the button is pressed as a form of visual feedback. However, compared to the KS11R2xCQD, it has a much taller profile which would mean that the PCB would need to be recessed further back into the torso.
