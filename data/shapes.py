
CTRL_SHAPES = {
    'ik': {
        'point': [[0.0, 1.0, 1.0],
                  [0.0, 1.0, -1.0],
                  [0.0, -1.0, -1.0],
                  [0.0, -1.0, 1.0],
                  [0.0, 1.0, 1.0]],
        'degree': 1},

    'fk': {
        'point': [[0.0, 0.7836, -0.7836],
                  [0.0, 1.1082, -0.0],
                  [-0.0, 0.7836, 0.7836],
                  [-0.0, 0.0, 1.1082],
                  [-0.0, -0.7836, 0.7836],
                  [-0.0, -1.1082, 0.0],
                  [0.0, -0.7836, -0.7836],
                  [0.0, -0.0, -1.1082],
                  [0.0, 0.7836, -0.7836],
                  [0.0, 1.1082, -0.0],
                  [-0.0, 0.7836, 0.7836]],
        'degree': 3,
        'knot': [-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]},

    'pole': {'point': [[0.0, 1.0, 0.0],
                       [0.0, -1.0, 0.0],
                       [0.0, 0.0, 0.0],
                       [-1.0, 0.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [0.0, 0.0, -1.0]],
             'degree': 1},

    'other': {'point': [[0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [0.0, -1.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0]],
              'degree': 1}
}
