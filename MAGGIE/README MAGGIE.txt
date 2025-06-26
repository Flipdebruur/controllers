#MAGGIE is a robot that has a stepper motor that moves a belt with a magnet on it so it can grab boxes with a magnetic strip
#this code needs to control said robot
#it will move on a predeterimed grid that we can measure 
#it has line sensors in the form of 5 ground sensors and 2 proximity sensons for obstacles
#it has 2 wheels and a swivel wheel that isnt powered just for balance
#it has to go the course pick up 4 boxes return through the course and unload the 4 boxes in 4 separate coordinates


#init
#grid Intersection as a 2d array
#costs same array as grid but the values will differ and be updated when an obstalce gets detected
#start goal and end goals 
#run initial dijkstra get initial route
#if on = True
#loop start
    #read sensors
    #encoder start
    #store grid coordinates
    #set goal to first point on path
    #go to next goal
    #linefollowing
    #if obstacle is detected and not near box goal
        # Obstacle Sequence:
            # 1. Stop motors immediately.
            # 2. Using encoder data that was counted from the last intersection return to said intersection .
            # 3. Turn 90 degrees to the right (or left, depending on strategy) using encoder data.
            # 4. Pathfind
    #if line lost
        #new goal calculated through odometry data, goes back to nearest line
    #If at intersection
        #new odometry counter activated from estimated 
        #if near box goal 
            #if touchsensor active:
                #activate magnet
                #sleep 0.3s
                #Boxstoring sequence
                #box counter incremented
        #if box counter < 3
            #next goal is next box
        #else
            #return original to starting coordinates
            #if at original coordinates initiate unloading sequence