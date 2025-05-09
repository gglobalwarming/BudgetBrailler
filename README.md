# BudgetBrailler
Open source, open hardware, low-cost electric brailler. This project is a work in progress, although I have produced (mostly) working prototypes which make very nice braille dots. I am a biologist not an engineer of any sort so I'm sure there are many improvements to be made. Any feedback to improve the design would be appreciated!

Braillers (aka Braille typewriters) are used by Blind or Visually Impaired persons to produce written text that can be "read" by touch based on patterns of dots. These devices can range from ~$800 for a mechanical Perkins Brailler to thousands of dollars for electric braillers. Haaving built several 3D printers from kits I wanted to see if I could design a low-cost brailler for a similar price to an entry level 3D printer. The  basic functional design is a set of rollers turned by a stepper motor to feed paper in and out, a toolhead on a linear rail driven by a stepper motor through a belt to move across lines of the paper, a linear actuator to imprint the dots on the page, and a keyboard for user input.

My goals in designing this embosser were as follow:
•	The embosser should cost $100-200. My original inspiration for this project was that if you can buy or build a capable 3D printer for that price, you should be able to do the same with a braille embosser.
•	The embosser should be open source and open hardware.
•	The embosser should be entirely 3D printed or made with off-the-shelf components.
•	The embosser should be simple. There should be as few parts as possible and, except for the electronics, no complex mechanisms.
•	The embosser should have an optional case to protect the user and the embosser.
•	The embosser should be well documented. This includes a bill of materials, detailed instructions, code, wiring diagrams, and stl and CAD files for all printed parts.

If you go looking you will find several similar projects but I found them to be either insufficiently documented or very complex to build. Add list/links to other projects. The complexity is not in and of itself a bad thing and may result in a more robust embosser, but that comes with a higher barrier of entry for building an embosser or repairing it if it becomes damaged. I think the only way my embosser fails on this account is in the electronics, although with basic soldering skills one should be able do the wiring themselves (source: I only have basic soldering skills).

Future Improvements:
•	Implement user feedback. Please let me know any problems or suggestions you have!
• Learn FreeCAD and remake all the 3D printed parts to move away from
•	Write code to allow the embosser to accept braille ASCII files to print
•	Implement QWERTY keyboard compatibility
•	Implement more auditory feedback from embosser
•	Detect when paper reaches the print line
• Improve paper feeding more generally
•	Make 3D printable gears as an option
•	Search for alternative electronics which are simpler, less expensive, or more reliable
•	Design PCBs to reduce wiring complexity
•	Design alternate print heads to make different size dots
•	Test different linear actuators
