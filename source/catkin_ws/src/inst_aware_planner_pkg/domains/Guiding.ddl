### Auto-generated planning domain from the Institution, Institution Domain and Grounding ###

(Domain Institution)
(Sensor command)
(ContextVariable Institution)
(Actuator mbot11)
(Actuator mbot03)
(SimpleOperator
(Head Institution::Guiding)
(RequiredState req1 mbot11::moveOnTrajectory(map))
(RequiredState req2 mbot03::moveInFormation(mbot11))
(RequiredState req3 command::Guiding)
(Constraint Equals(req1,req2))
)
