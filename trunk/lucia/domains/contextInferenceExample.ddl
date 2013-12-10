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

(SimpleDomain ContextInferenceExample)

(Sensor Location)
(Sensor Stove)
(Sensor Chair)
(Sensor Door)
(Sensor Fridge)

(ContextVariable Human)

(SimpleOperator
 (Head Human::Cooking())
 (RequiredState inkitchen Location::Kitchen())
 (RequiredState fridgeopen Fridge::Open())
 (Constraint Starts(Head,inkitchen))
 (Constraint Contains(Head,fridgeopen))
)
