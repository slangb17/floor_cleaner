This package is build by C2-18, 1. semester Robotics students at Aalborg University.

The package floor_cleaner includes three nodes: node_maker, path_base, and path_maker.

	1) Node_maker is used to publish points manually to the topic /cleaningpoints.
	2) Path_base is used for following walls and mapping the corners.
	3) Path_maker creates the markers and sends goal the move_base action server.
