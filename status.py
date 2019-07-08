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
        # from behaviours.final_demo.orientation import Definitions, assign_orientation

        # map start to end node with their outcome vectors
        outcomes = {('mid_left', 'table1_left')     : [self.graph.positive_x_axis],#
                    ('mid_left', 'table2')          : [self.graph.negative_x_axis],#
                    ('mid_left', 'mid_right')       : [self.graph.positive_x_axis],#
                    ('mid_left', 'mid_center')      : [self.graph.positive_x_axis],#
                    ('table2', 'mid_left')          : [self.graph.negative_y_axis, self.graph.positive_x_axis],##
                    ('table1_left', 'mid_left')     : [self.graph.negative_y_axis],##
                    ('mid_right', 'mid_left')       : [self.graph.negative_x_axis],#
                    ('mid_right', 'table1_right')   : [self.graph.negative_x_axis],#
                    ('mid_right', 'mid_center')     : [self.graph.negative_x_axis],#
                    ('table1_right', 'mid_right')   : [self.graph.negative_y_axis],#
                    ('mid_center', 'mid_left')      : [self.graph.negative_x_axis],#
                    ('mid_center', 'mid_right')     : [self.graph.positive_x_axis],#
                    ('mid_center', 'drop-off')      : [self.graph.negative_y_axis],#
                    ('drop-off', 'mid_center')      : [self.graph.positive_y_axis],#
        }

        def foo(node_dict):
            for prop in ['positive_x_axis', 'positive_y_axis', 'negative_x_axis', 'negative_y_axis']:
                if self.graph.__dict__[prop] == node_dict:
                    return prop

        succeed = True
        for start_node in self.graph.connections.keys():
            for end_node in self.graph.connections[start_node]:
                if start_node == 'start':
                    continue

                start = self.graph.waypoint_of(start_node)#['position']
                end = self.graph.waypoint_of(end_node)#['position']
                output = self.graph.assign_orientation(start, end)
                # output = assign_orientation(start, end, self.graph.waypoint_of(end_node)['name'])

                expected_outcome = output in outcomes[(start_node, end_node)]
                if not expected_outcome:
                    succeed = False
                    self.vis.edge(start_node, end_node, color='red')
                    print('FAILED: start', start_node, 'end', end_node, 'output', foo(output), 'expected_output', [foo(out) for out in outcomes[(start_node, end_node)]])
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
