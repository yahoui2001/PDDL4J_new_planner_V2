(define (domain robot_navigation)
    (:requirements :strips :typing)
    (:types location robot)

    (:predicates
        (at ?r - robot ?l - location)
        (connected ?l1 ?l2 - location)
    )

    (:action move
        :parameters (?r - robot ?from ?to - location)
        :precondition (and (at ?r ?from) (connected ?from ?to))
        :effect (and (at ?r ?to) (not (at ?r ?from)))
    )
)

