##################
# Reserved words #
#################################################################
#                                                               #
#   Head                                                        #
#   Resource                                                    #
#   Sensor                                                      #
#   ContextVariable                                             #
#   SimpleOperator                                              #
#   SimpleDomain                                                #
#   Constraint                                                  #
#   RequiredState						#
#   AchievedState						#
#   RequriedResoruce						#
#   All AllenIntervalConstraint types                           #
#   '[' and ']' should be used only for constraint bounds       #
#   '(' and ')' are used for parsing                            #
#                                                               #
#################################################################

(SimpleDomain TestProactivePlanningLucia)

(Sensor Location)
(Sensor Stove)
(Sensor Chair)
(Sensor Door)
(Sensor Fridge)

(ContextVariable Human)
(ContextVariable Alarm)

# WORKS
#(SimpleOperator
# (Head Human::Cooking())
# (RequiredState inkitchen Location::Kitchen())
# (RequiredState fridgeopen Fridge::Open())
# (Constraint Starts(Head,inkitchen))
# (Constraint Contains(Head,fridgeopen))
#)

(SimpleOperator
 (Head Human::Cooking())
 (RequiredState inkitchen Location::Kitchen())
 (RequiredState stoveon Stove::On())
 (Constraint StartsOrStartedBy(stoveon,Head))
 (Constraint Finishes(Head,inkitchen))
)

(SimpleOperator
 (Head Human::Eating())
 (RequiredState indiningroom Location::DiningRoom())
 (RequiredState cooking Human::Cooking())
 (RequiredState chairoccupied Chair::Occupied())
# (RequiredState saybonappetit Robot::SayBonAppetit())
 (Constraint After(Head,cooking))
 (Constraint Equals(Head,chairoccupied))
 (Constraint Finishes(Head,indiningroom))
)

(SimpleOperator
 (Head Alarm::FireHazard())
 (RequiredState stoveon Stove::On())
 (RequiredState eating Human::Eating())
 (RequiredState saywarning Robot::SayWarning())
 (Constraint StartsOrStartedBy(eating,Head))
 (Constraint Finishes(Head,stoveon))
)

(SimpleOperator
 (Head Robot::SayBonAppetit())
 (RequiredState reachperson Robot::MoveTo())
 (Constraint MetBy(Head,reachperson))
 (Constraint Duration[2000,INF](Head))
)

(SimpleOperator
 (Head Robot::SayWarning())
 (RequiredState reachperson Robot::MoveTo())
 (Constraint MetBy(Head,reachperson))
 (Constraint Duration[2000,INF](Head))
)

(SimpleOperator
 (Head Robot::MoveTo())
 (Constraint Duration[2000,INF](Head))
)