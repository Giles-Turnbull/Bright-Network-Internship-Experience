Bright Network Internship, June 2022
# Bright-Network-Internship-Experience
Bright Network's Internship Experience coding challenge

Overview:
In this challenge, you are going to implement Amazon’s pathfinding algorithm for Amazon’s self-driving delivery vehicles.
The self-driving vehicle will need to create a path on a 2D-grid that contains a starting point (x,y), a delivery point (x,y)
and a number of obstacles. Your vehicle can navigate to any of the adjacent squares (even diagonally), as long as the
squares are inbound and do not contain an obstacle.

General notes:
You can use any language and ideally the output is to a command line

Phase 1:
Implement a 10x10 grid that contains a starting and delivery point. The following obstacles are to be added in these locations:
(9, 7) (8, 7) (6, 7) (6, 8)
Your algorithm should calculate a valid path avoiding the obstacles and reaching the delivery point.

Phase 2:
Add an aditional 20 obstacles in random locations and print their locations using the format [(x1, y1), (x2, y2),...].
The obstacles should not overlap existing ones and should not be placed on the starting and delivery points.
The algorithm should calculate a valid path avoiding the obstacles and reaching the delivery point.
Your solution should print the path in the format [(x1, y1), (x2, y2),...].

Bonus:
In the event that your vehicle is unable to reach its destination, your algorithm should print "Unable to reach delivery point".
It should then identify which obstacles should be removed in order for the vehicle to reach it's destination.

The algorithm should suggest the least amount of obstacles using the format [(x1, y1), (x2, y2),...] in order for your vehicle to reach its destination.