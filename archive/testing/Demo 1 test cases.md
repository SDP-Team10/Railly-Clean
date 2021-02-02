Test cases to cover all functionality that's to be achieved by the first demo


**Functions in project report**

* RobotStability
  * Robot remains upright with no arm
  * Robot remains upright with arm at 'minimum extension'
  * Robot remains upright with arm at 'maximum extension'
  * Robot remains upright during simulated 'movement' of the train (no arm)
  * Robot remains upright during simulated 'movement' of the train arm 'minimum extension'
  * Robot remains upright during simulated 'movement' of the train arm 'maximum extension' 

* RobotMovement
  * Robot moves through carriage without collisions with seating area starting 1m away from end
  * Robot moves through carriage without collisions with seating area starting 5m away from end
  * Robot doesn't collide with unexpected obstacles in front of it
 
* CarriageEndRecognition
  * Robot identifies carriage end when immediately in front of it
  * Robot identifies carriage end from 1m 
  * Robot identifies carriage end from 5m
  * Robot stop before carriage end
  * Robot starts facing 90 degrees to end - > identify carriage end from 1m
  * Robot starts facing 180 degrees to end - > identify carriage end from 1m
  * Robot starts facing 90 degrees to end - > identify carriage end from 5m
  * Robot starts facing 180 degrees to end - > identify carriage end from 5m
  * Robot identifies obstacle which is not carriage end when immediately in front of it
  * Robot detects obstacle which is not carriage end from 1m
  * Robot detects obstacle which is not carriage end from 5m
  
