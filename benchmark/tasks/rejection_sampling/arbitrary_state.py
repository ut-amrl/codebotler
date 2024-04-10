from benchmark.simulator import State

arbitrary_state = State().addLocation(".*").addLocation(".*").addLocation(".*").addRobotLocation("start_loc").addAgent(".*", ".*", [r".*"]).addObject(".*", ".*")