import quadrotor.command as cmd
from math import sqrt

def plan_mission(mission):

    # this is an example illustrating the different motion commands,
    # replace them with your own commands and activate all beacons
    commands  = [
        cmd.up(1),
        cmd.turn_left(90-26.57),
        cmd.forward(sqrt((2*0.5)**2+(4*0.5)**2)),
        cmd.turn_right(90-26.57),
        cmd.forward(4),
        cmd.right(2),
        cmd.backward(4),
        cmd.right(2),
        cmd.forward(4),
        cmd.turn_left(45),
    ]

    mission.add_commands(commands)
