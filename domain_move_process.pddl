(define (domain turtlebot)
    ; Sin approximation (Bhaskara I, deg)
    ; Numerator: 4a(180-a) -> (* 4 (* (a ?r) (- 180 (a ?r))) )
    ; Denominator: 40500 - a(180-a) -> (- 40500 (* (a ?r) (- 180 (a ?r))) )
    ; Num/Den: (/ (* 4 (* (a ?r) (- 180 (a ?r)))) (- 40500 (* (a ?r) (- 180 (a ?r)))) )

    ; Cos approximation (Bhaskara I phase shifted, deg)
    ; Numerator: 32400 - 4a^2 -> (- 32400 (* 4 (^ (a ?r) 2)) )
    ; Denominator: 32400 + a^2 -> (+ 32400 (^ (a ?r) 2) )
    ; Num/Den: (/ (- 32400 (* 4 (^ (a ?r) 2)) ) (+ 32400 (^ (a ?r) 2) ) )

    (:requirements :typing :fluents)
    (:types robot obstacle)

    ;; Functions / Fluents
    (:functions
        (maxx)                ; Map boundaries
        (maxy)
        (minx)
        (miny)

        (x ?r -robot)         ; Robot coordinates
        (y ?r -robot)
        (a ?r -robot)
        (vx ?r -robot)        ; Robot forward velocity
        (va ?r -robot)        ; Robot angular velocity (constant)
        (ax ?r -robot)        ; Robot forward acceleration (constant)

        (max_vel_x ?r -robot)        ; Robot velocity limits
        
        (cx ?o -obstacle)     ; Obstacle center coordinates
        (cy ?o -obstacle)
        (rcx ?o -obstacle)    ; Obstacle clearance radii
        (rcy ?o -obstacle)
        (rex ?o -obstacle)    ; Obstacle energy radii
        (rey ?o -obstacle)    
    )

    ; Processes

    ;; Not testing obstacle avoidance for now
    (:process move_forward
        :parameters (?r -robot)
        :precondition()
        ; :precondition(and
        ;                 (forall (?o -obstacle) 
        ;                     (not 
        ;                         (and  
        ;                             (<= (^ (^ (- (x ?r) (cx ?o)) 2) 0.5) (rcx ?o))
        ;                             (<= (^ (^ (- (y ?r) (cy ?o)) 2) 0.5) (rcy ?o))
        ;                         )
        ;                     )
        ;                )
        ;           )

        ;; move in x -> vx*cos(a)
        ;; move in y -> vx*sin(a)
        :effect (and (increase (x ?r) (* (* #t (vx ?r)) (/ (- 32400 (* 4 (^ (a ?r) 2)) ) (+ 32400 (^ (a ?r) 2)))) )
                     (increase (y ?r) (* (* #t (vx ?r)) (/ (* 4 (* (a ?r) (- 180 (a ?r)))) (- 40500 (* (a ?r) (- 180 (a ?r)))))) ) 
                )
    )

    ; (:process turn
    ;     :parameters (?r -robot)
    ;     :precondition()
    ;     :effect (increase (a ?r) (* #t (va ?r)))
    ; )

    ;; Constraints
    ; Stay within map bounds
    (:constraint map_bounds
        :parameters (?r -robot)
        :condition (and 
                        (and 
                            (<= (x ?r) (maxx)) 
                            (<= (y ?r) (maxy))
                        ) 
                        (and 
                            (>= (x ?r) (minx)) 
                            (>= (y ?r) (miny))
                        )
                   )
    )

    ; Robot speed limit
    ; (:constraint vel_limit
    ;     :parameters (?r -robot)
    ;     :condition (and
    ;                     (and (>= (vx ?r) (* -1 (max_vel_x ?r))) (<= (vx ?r) (max_vel_x ?r)))
    ;                     ; (and (>= (va ?r) (* -1 (max_vel_a ?r))) (<= (va ?r) (max_vel_a ?r)))
    ;                 )
    ; )

    ; Constrain robot orientation between 0~360 deg
    ; (:constraint a_lim
    ;     :parameters (?r -robot)
    ;     :condition (and (>= (a ?r) 0) (<= (a ?r) 360))
    ; )

    ;; Actions
    (:action inc_vel_x
        :parameters (?r -robot)
        :precondition ()
        :effect (increase (vx ?r) (ax ?r))
    )

    (:action dec_vel_x
        :parameters (?r -robot)
        :precondition ()
        :effect (decrease (vx ?r) (ax ?r))
    )

    ;; Turn counter-clockwise (positive rotation)
    (:action turn_ccw
        :parameters (?r -robot)
        :precondition ()
        :effect (increase (a ?r) (va ?r))
    )

    ;; Turn clockwise (negative rotation)
    (:action turn_cw
        :parameters (?r -robot)
        :precondition ()
        :effect (decrease (a ?r) (va ?r))
    )

    ; (:action inc_vel_a
    ;     :parameters (?r -robot)
    ;     :precondition ()
    ;     :effect (increase (va ?r) 90)
    ; )

    ; (:action dec_vel_a
    ;     :parameters (?r -robot)
    ;     :precondition ()
    ;     :effect (decrease (va ?r) 90)
    ; )
)