(define (problem turtle_problem)
	(:domain turtlebot)

	(:objects
			turtle -robot
			 o1 -obstacle
			 o2 -obstacle
			 o3 -obstacle
			 o4 -obstacle
			 o5 -obstacle
			 o6 -obstacle
			 o7 -obstacle
	)

	(:init
			;;;;;; All [x,y] coordinates are in grid units
			;;;;;; All orientation/angle coordinates are in degrees

			;; Map limit constraints
			;; Keeps the plan within the map boundaries
			(= (maxx) 48.0)
			(= (maxy) 60.0)
			(= (minx) 0.0)
			(= (miny) 0.0)

			;; Obstacles' parameters
			;; [cx, cy] = obstacle center
			;; [rcx, rcy] = collision radius x and y (robot must keep away to avoid collision)
			;; [rex, rey] = energy radius x and y (robot must avoid to keep from turning sensors on)
			(= (cx o1) 0.5)
			(= (cy o1) 28.5)
			(= (rcx o1) 4.0)
			(= (rcy o1) 31.5)
			(= (rex o1) 7.0)
			(= (rey o1) 34.5)

			(= (cx o2) 46.5)
			(= (cy o2) 28.5)
			(= (rcx o2) 4.0)
			(= (rcy o2) 31.5)
			(= (rex o2) 7.0)
			(= (rey o2) 34.5)

			(= (cx o3) 24.0)
			(= (cy o3) 0.5)
			(= (rcx o3) 26.0)
			(= (rcy o3) 4.0)
			(= (rex o3) 30.0)
			(= (rey o3) 7.0)

			(= (cx o4) 24.0)
			(= (cy o4) 58.5)
			(= (rcx o4) 26.0)
			(= (rcy o4) 4.0)
			(= (rex o4) 30.0)
			(= (rey o4) 7.0)

			(= (cx o5) 24.5)
			(= (cy o5) 53.0)
			(= (rcx o5) 8.0)
			(= (rcy o5) 8.5)
			(= (rex o5) 14.0)
			(= (rey o5) 14.5)

			(= (cx o6) 24.5)
			(= (cy o6) 20.5)
			(= (rcx o6) 7.0)
			(= (rcy o6) 7.0)
			(= (rex o6) 12.0)
			(= (rey o6) 12.0)

			(= (cx o7) 40.5)
			(= (cy o7) 5.5)
			(= (rcx o7) 7.0)
			(= (rcy o7) 7.0)
			(= (rex o7) 12.0)
			(= (rey o7) 12.0)


			;; Robot parameters
			;; [x,y,a] = robot pose, [x,y] position and a orientation
			;; [vx,ax] = robot linear frontal velocity (variable) and acceleration (constant)
			;; [va] = robot angular velocity (constant)
			;; [max_vel_x, max_vel_a] = robot velocity limits
			(= (x turtle) 10)
			(= (y turtle) 10)
			(= (a turtle) 0)
			(= (vx turtle) 0)
			(= (ax turtle) 1)
			(= (va turtle) 90)
			(= (max_vel_x turtle) 3)
			(= (max_vel_a turtle) 90)
	)

	;; Goal is robot at goal position, facing left (orientation = 0) with null velocities (a.k.a. stopped)
	(:goal
			(and
				;; Opposite corner of the map, this is the hard problem (~ 4.4 mins to plan)
				; (= (x turtle) 37)
				; (= (y turtle) 46)

				;; Literally one block diagonal from the robot, this is the easy sanity check problem
				(= (x turtle) 11)
				(= (y turtle) 11)

				;; Robot facing left
				(= (a turtle) 0)
				
				;; Robot stopped
				(= (vx turtle) 0)
				; (= (va turtle) 0)
			)
	)
)