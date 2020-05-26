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
			(= (resolution) 1.0)
			(= (maxx) 48.0)
			(= (maxy) 60.0)
			(= (minx) 0.0)
			(= (miny) 0.0)

			(= (cx o1) 0.5)
			(= (cy o1) 28.5)
			(= (rc1 o1) 4.0)
			(= (rc2 o1) 31.5)
			(= (re1 o1) 7.0)
			(= (re2 o1) 34.5)

			(= (cx o2) 46.5)
			(= (cy o2) 28.5)
			(= (rc1 o2) 4.0)
			(= (rc2 o2) 31.5)
			(= (re1 o2) 7.0)
			(= (re2 o2) 34.5)

			(= (cx o3) 24.0)
			(= (cy o3) 0.5)
			(= (rc1 o3) 26.0)
			(= (rc2 o3) 4.0)
			(= (re1 o3) 30.0)
			(= (re2 o3) 7.0)

			(= (cx o4) 24.0)
			(= (cy o4) 58.5)
			(= (rc1 o4) 26.0)
			(= (rc2 o4) 4.0)
			(= (re1 o4) 30.0)
			(= (re2 o4) 7.0)

			(= (cx o5) 24.5)
			(= (cy o5) 53.0)
			(= (rc1 o5) 8.0)
			(= (rc2 o5) 8.5)
			(= (re1 o5) 14.0)
			(= (re2 o5) 14.5)

			(= (cx o6) 24.5)
			(= (cy o6) 20.5)
			(= (rc1 o6) 7.0)
			(= (rc2 o6) 7.0)
			(= (re1 o6) 12.0)
			(= (re2 o6) 12.0)

			(= (cx o7) 40.5)
			(= (cy o7) 5.5)
			(= (rc1 o7) 7.0)
			(= (rc2 o7) 7.0)
			(= (re1 o7) 12.0)
			(= (re2 o7) 12.0)

			(= (x turtle) 10)
			(= (y turtle) 10)

			(= (energy turtle) 0)
	)

	(:goal
			(and
				(= (x turtle) 37)
				(= (y turtle) 49)
				(= (energy turtle) 0)
			)
	)
	(:metric minimize (energy turtle))
)