### Auto-generated planning domain from the Institution, Institution Domain and Grounding ###

(Domain Institution)
(Sensor command)
(ContextVariable Institution)
(Actuator pepper01)
(Actuator mbot11)
(SimpleOperator
(Head Institution::Touring)
(RequiredState req1 pepper01::describe(lab))
(RequiredState req2 mbot11::wayPointNavigation(next_to_nao,yellow_sphere,red_box))
(RequiredState req3 command::Touring)
(Constraint Equals(req1,req2))
)
