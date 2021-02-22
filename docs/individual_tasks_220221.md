### Daniel - 
For the next milestone we would like the robot to be able to collect trash inside itself like a bin, would you mind working on a new model which includes an opening hatch for example on the side of the robot (same side as the arm moves) so that it can sweep the rubbish inside itself (there would be a bag of some sort attached so that it can be emptied). For the rubbish/valuables detection we also need a camera which is a bit higher up which means we will probably need more height on this new model too to allow the camera to be placed in such a way that it can see the whole table.

### Arnav - 
I'd like you to take charge of testing this week.  If you haven't already, it might help to watch the recordings of the quantitative analysis workshop given by Barabara Webb a couple of weeks ago. Have you managed to figure out table painting? From what I can tell, you have to add a Pen node as a child of whatever you want to do the painting, so for us adding it to the arm head would probably work? Did you make the different world with different table lengths for testing?

### Sean - 
Work on making the robot move properly for cleaning the table when the table is on the left (so make it move backwards and start from the back of the table. Once that's finished. The robot should be ready for testing (fingers crossed) so you can work with Arnav on Testing from Wednesday.

### Austin - 
Make the arm controller use the distance sensors to get the table length (pass in the distance + length of robot edge to arm beginning to the kinematics for example).  Then, if you get time, can you see how the arm gets on a pushing rubbish off the table (add some rubbish nodes on top of the table to check if they move). Additionally, video editing for the report.

### Mate - 
Get some images of rubbish/not trash on the tables in Webots for them to test their algorithm on to make sure it works with the webots environment. Then continue working with Apurv on making that work or adapting it to button detection 

### Apurv - 
1. Make any necessary changes to the rubbish detection if it doesn't work with webots images 2. If it works well and no changes needed, see if you can make a start on the button detection this week to save on time next week 

### Caitlin, Suhas, Anh - 
Demo report, slides, video
