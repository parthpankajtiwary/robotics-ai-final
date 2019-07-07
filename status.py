import unittest

class TestNavigation(unittest.TestCase):
    # __init__(self):

    def setUp(self):
        from graphviz import Digraph
        from behaviours.final_demo.parse_graph import Graph
        import yaml
        import graphviz as gv
        import sys

        dirname = './behaviours/move_base/'
        graph_nodes = None
        with open(dirname + 'simulation_waypoints.yaml', 'r') as f:
            graph_nodes = yaml.load(f)
            # self.graph_nodes = yaml.load(f)

        self.graph = Graph(graph_nodes, fname='./behaviours/final_demo/alice_graph.dot')

        self.vis = Digraph(comment='Navigation orientation test')
        for node in self.graph.nodes():
            self.vis.node(node)

    def test_dynamic_rotations(self):
        from behaviours.final_demo.orientation import Definitions, assign_orientation

        # map start to end node with their outcome vectors
        outcomes = {('mid_left', 'table1_left')     : Definitions.positive_x_axis,#
                    ('mid_left', 'table2')          : Definitions.negative_x_axis,#
                    ('mid_left', 'mid_right')       : Definitions.positive_x_axis,#
                    ('mid_left', 'mid_center')      : Definitions.positive_x_axis,#
                    ('table2', 'mid_left')          : Definitions.negative_y_axis,##
                    ('table1_left', 'mid_left')     : Definitions.negative_y_axis,##
                    ('mid_right', 'mid_left')       : Definitions.negative_x_axis,#
                    ('mid_right', 'table1_right')   : Definitions.positive_y_axis,#
                    ('mid_right', 'mid_center')     : Definitions.negative_x_axis,#
                    ('table1_right', 'mid_right')   : Definitions.negative_y_axis,#
                    ('mid_center', 'mid_left')      : Definitions.negative_x_axis,#
                    ('mid_center', 'mid_right')     : Definitions.positive_x_axis,#
                    ('mid_center', 'drop-off')      : Definitions.negative_x_axis,#
                    ('drop-off', 'mid_center')      : Definitions.positive_x_axis,#
        }

        succeed = True
        for start_node in self.graph.connections.keys():
            for end_node in self.graph.connections[start_node]:
                if start_node == 'start':
                    continue

                start = self.graph.waypoint_of(start_node)['position']
                end = self.graph.waypoint_of(end_node)['position']
                output = assign_orientation(start, end, self.graph.waypoint_of(end_node)['name'])

                expected_outcome = output == outcomes[(start_node, end_node)]
                if not expected_outcome:
                    succeed = False
                    self.vis.edge(start_node, end_node, color='red')
                    print('FAILED: start', start_node, 'end', end_node, 'output', output, 'expected_output', outcomes[(start_node, end_node)])
                else:
                    self.vis.edge(start_node, end_node, color='black')

        self.vis.view()
        self.assertTrue(succeed)

class TestNetwork(unittest.TestCase):
    pass
    # def test_images(self):
    #     self.assertEqual(input, output)

class TestMainBehaviour(unittest.TestCase):
    pass

if __name__ == '__main__':
    unittest.main()
