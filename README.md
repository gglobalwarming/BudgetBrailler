# BudgetBrailler
Open source, open hardware, low-cost electric brailler. This project is a work in progress, although I have produced (mostly) working prototypes which make very nice braille dots. I am a biologist not an engineer of any sort so I'm sure there are many improvements to be made. Any feedback to improve the design would be appreciated!  

## Introduction
Braillers (aka Braille typewriters) are used by Blind or Visually Impaired persons to produce written text that can be "read" by touch based on patterns of dots. These devices can range from ~$800 for a mechanical Perkins Brailler to thousands of dollars for electric braillers. Having built several 3D printers from kits I wanted to see if I could design a low-cost brailler for a similar price to an entry level 3D printer. The  basic functional design is a set of rollers turned by a stepper motor to feed paper in and out, a toolhead on a linear rail driven by a stepper motor through a belt to move across lines of the paper, a linear actuator to imprint the dots on the page, and a keyboard for user input.  

## Design Goals
* The embosser should cost $100-200. My original inspiration for this project was that if you can buy or build a capable 3D printer for that price, you should be able to do the same with a braille embosser.  
* The embosser should be open source and open hardware.  
* The embosser should be entirely 3D printed or made with off-the-shelf components.  
* The embosser should be simple. There should be as few parts as possible and, except for the electronics, no complex mechanisms.  
* The embosser should have an optional case to protect the user and the embosser.  
* The embosser should be well documented. This includes a bill of materials, detailed instructions, code, wiring diagrams, and stl and CAD files for all printed parts.  

## Limitations
* Custom keyboard  
* Feeding paper is imprecise  
* Manually aligning first line on page required  
* Printing parts requires 256mm print bed  
* Long term durability not tested  
* Very little auditory feedback  
* Manual wiring and crimping needed
* No screen for high vision users

## Future Improvements:
* Implement user feedback. Please let me know any problems or suggestions you have!  
* Learn FreeCAD and remake all the 3D printed parts to move away from SolidWorks  
* Write code to allow the embosser to accept braille ASCII files to print  
* Implement QWERTY keyboard compatibility  
* Redesign custom keyboard to have its own microcontroller and memory  
* Allow keyboard to be used separately as a note taker then later connected for printing  
* Implement more auditory feedback from embosser  
* Detect when paper reaches the print line  
* Improve paper feeding more generally  
* Make 3D printable gears as an option  
* Search for alternative electronics which are simpler, less expensive, or more reliable  
* Design PCBs to reduce wiring complexity  
* Design alternate print heads to make different size dots
* Invert printing so that braille can be read right-side up without removing paper
* Add optical snsor to auto-align page for first line printing  
* Test different linear actuators  

## Similar Projects
There are several similar open source braillers, although they are directed at printing from files rather than being used like a typewriter. The top two are linked below:  
[BrailleRap](https://www.braillerap.org/en/)  
[OpenBraille](https://github.com/carloscamposalcocer/OpenBraille)  
