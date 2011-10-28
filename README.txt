
This repository is a branch of ArduPilot Mega.
http://code.google.com/p/ardupilot-mega/

It is maintained by Pat Hickey (pat@moreproductive.org) to support Purple
hardware.

Notes:
- It is very alpha and not flight safe. Please test carefully in a controlled
  environment before flying outdoors, and please only fly outdoors away from
  bystanders!
- At the moment, there is some deprecated code sitting around that hasn't been
  cleaned out.
- At the moment, ArduPlane is probably quite broken. I've only been maintaining
  and testing with the ArduCopter build, though nothing should prevent ArduPlane
  from getting brought up to speed.

Many thanks to Jose Julio, Andrew Tridgell, Chris Anderson, Jordi Munoz,
Michael Smith, and others for help with this work!

================

Building using arduino
--------------------------
To install the libraries:
 - copy Library Directories to your \arduino\hardware\libraries\ or arduino\libraries directory
 - Restart arduino IDE

 * Each library comes with a simple example. You can find the examples in menu File->Examples

Building using make 
-----------------------------------------------
 - go to directory of sketch and type make.

Building using cmake
-----------------------------------------------
 - mkdir build
 - cd build
 - cmake ..
 - make (will build every sketch)
 - make ArduPlane (will build just ArduPlane etc.)

Build a package using cpack
-----------------------------------------------
 - cd build
 - cmake ..
 - make package
 - make package_source
