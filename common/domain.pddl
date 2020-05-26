(define (domain turtlebot)
    (:requirements :typing :fluents)
    (:types robot obstacle)

    ;; Functions / Fluents
    (:functions
        (maxx)                ; Map boundaries
        (maxy)
        (minx)
        (miny)
        (resolution)          ; Map resolution

        (x ?r -robot)         ; Robot coordinates
        (y ?r -robot)
        (energy ?r -robot)    ; Robot energy
        
        (cx ?o -obstacle)     ; Obstacle center coordinates
        (cy ?o -obstacle)
        (rc1 ?o -obstacle)    ; Obstacle clearance x radius
        (rc2 ?o -obstacle)    ; Obstacle clearance y radius
        (re1 ?o -obstacle)    ; Obstacle energy x radius
        (re2 ?o -obstacle)    ; Obstacle energy y radius
    )

    ;; Constraints
    ; Stay within map bounds
    (:constraint map_bounds
        :parameters (?r -robot)
        :condition (and 
                        (and 
                            (<= (x ?r) (/ (maxx) (resolution))) 
                            (<= (y ?r) (/ (maxy) (resolution)))
                        ) 
                        (and 
                            (>= (x ?r) (/ (minx) (resolution))) 
                            (>= (y ?r) (/ (miny) (resolution)))
                        )
                   )
    )

    ; Energy should be more than 0 to avoid the planner from running wild
    (:constraint energy_range
        :parameters (?r -robot)
        :condition(>= (energy ?r) 0)
    )

    ;; Actions
    ; Movement with low energy
    (:action move_up_low_e
     :parameters (?r -robot)
     :precondition(and
                        (<= (energy ?r) 0)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution))) 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (rc1 ?o))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect(increase (y ?r) 1)
    )

    (:action move_up_right_low_e
     :parameters (?r -robot)
     :precondition(and
                        (<= (energy ?r) 0)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (+ (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect(and 
                (increase (x ?r) 1)
                (increase (y ?r) 1)
            )
    )

    (:action move_up_left_low_e
     :parameters (?r -robot)
     :precondition(and
                        (<= (energy ?r) 0)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (+ (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect(and 
                (decrease (x ?r) 1)
                (increase (y ?r) 1)
            )
    )
    
    (:action move_down_low_e
     :parameters (?r -robot)
     :precondition(and 
                       (<= (energy ?r) 0)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution))) 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (rc1 ?o))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect (decrease (y ?r) 1)
    )

    (:action move_down_left_low_e
     :parameters (?r -robot)
     :precondition(and
                        (<= (energy ?r) 0)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (- (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect(and 
                (decrease (x ?r) 1)
                (decrease (y ?r) 1)
            )
    )

    (:action move_down_right_low_e
     :parameters (?r -robot)
     :precondition(and
                        (<= (energy ?r) 0)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (- (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect(and 
                (increase (x ?r) 1)
                (decrease (y ?r) 1)
            )
    )

    (:action move_right_low_e
     :parameters (?r -robot)
     :precondition(and 
                       (<= (energy ?r) 0)
                       (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (x ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution))) 
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (rc2 ?o))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect (increase (x ?r) 1)
    )
    
    (:action move_left_low_e
     :parameters (?r -robot)
     :precondition(and 
                       (<= (energy ?r) 0)
                       (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (x ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution))) 
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (rc2 ?o))
                                )
                            )
                       )
                       (forall (?o -obstacle)
                            (not 
                                (and 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                                )
                            )
                       )
                  )
     :effect (decrease (x ?r) 1)
    )

    ; Movement with high energy
    (:action move_up_high_e
     :parameters (?r -robot)
     :precondition(and
                        (> (energy ?r) 1)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution))) 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (rc1 ?o))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect (increase (y ?r) 1)
    )

    (:action move_up_right_high_e
     :parameters (?r -robot)
     :precondition(and
                        (> (energy ?r) 1)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (+ (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect(and 
                (increase (x ?r) 1)
                (increase (y ?r) 1)
            )
    )

    (:action move_up_left_high_e
     :parameters (?r -robot)
     :precondition(and
                        (> (energy ?r) 1)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (+ (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect(and 
                (decrease (x ?r) 1)
                (increase (y ?r) 1)
            )
    )
    
    (:action move_down_high_e
     :parameters (?r -robot)
     :precondition(and 
                       (> (energy ?r) 1)
                       (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution))) 
                                    (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (rc1 ?o))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect (decrease (y ?r) 1)
    )

    (:action move_down_left_low_e
     :parameters (?r -robot)
     :precondition(and
                        (> (energy ?r) 1)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (- (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect(and 
                (decrease (x ?r) 1)
                (decrease (y ?r) 1)
            )
    )

    (:action move_down_right_low_e
     :parameters (?r -robot)
     :precondition(and
                        (> (energy ?r) 1)
                        (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution)))
                                    (<= (^ (^ (- (- (y ?r) 1) (* (cy ?o) (resolution))) 2) 0.5) (* (rc2 ?o) (resolution)))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect(and 
                (increase (x ?r) 1)
                (decrease (y ?r) 1)
            )
    )

    (:action move_right_high_e
     :parameters (?r -robot)
     :precondition(and 
                       (> (energy ?r) 1)
                       (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (+ (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution))) 
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (rc2 ?o))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect (increase (x ?r) 1)
    )
    
    (:action move_left_high_e
     :parameters (?r -robot)
     :precondition(and 
                       (> (energy ?r) 1)
                       (forall (?o -obstacle) 
                            (not 
                                (and 
                                    (<= (^ (^ (- (- (x ?r) 1) (* (cx ?o) (resolution))) 2) 0.5) (* (rc1 ?o) (resolution))) 
                                    (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (rc2 ?o))
                                )
                            )
                       )
                       (exists (?o -obstacle)
                            (and 
                                (<= (^ (^ (- (x ?r) (* (cx ?o) (resolution))) 2) 0.5) (re1 ?o))
                                (<= (^ (^ (- (y ?r) (* (cy ?o) (resolution))) 2) 0.5) (re2 ?o))                    
                            )
                       )
                  )
     :effect (decrease (x ?r) 1)
    )

    ; energy actions
    (:action turn_on_sensor
     :parameters(?r -robot)
     :precondition()
     :effect(increase (energy ?r) 1)
    )

    (:action turn_off_sensor
     :parameters(?r -robot)
     :precondition()
     :effect(decrease (energy ?r) 1)
    )    
)
