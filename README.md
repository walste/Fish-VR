Fish-VR
=======

OS: Linux, Ubuntu 12.04

A virtual reality assay for larval zebrafish including code for optokinetic (OKR) and optomotive reflex (OMR), simple Prey capture (PC) and a classical conditioning assay with a visual conditioned stimulus.

Additional equipment: 
---------------------
Beamer (Optoma), Camera (Point Grey Research), Infrared illumination /w bandpass filter, laser (Roithner), Arduino Mega board. 

Additional software: 
-------------------
ROS for creating the visual stimulus, Motmot (Straw, A.D. and Dickinson, M.H. (2009))
for communicating with the camera, Arduino software /w pyFirmata for communicating with the laser

Stimulus:
---------
A bar grating (red or white with black) which is moving in the OKR and OMR assay and standing in the classical conditioning assay. 
A number of spheres, which oscillate in front of the fish.
The unconditioned stimulus in the classical conditioning assay is a laser. 

Set-up:
-------
The larval zebrafish (6-10dpf) are immobilized in low melting point agarose (2%) in an inverted lid of a 35 mm petri dish and the agarose around tail and head and in the fish's field of view is removed with a scalpel. A projection screen is attached to the rim of the petri dish and the dish is mounted on a stage. The visual stimulus is projected from the front at a distance of about 35 cm while the fish is being filmed from below, using a mirror. An ifrared LED illuminates the fish from the side. 


