pi = 3.14159265358979
CLASS = {
    0:  {'name': 'a_L_shape_angle'      , 'type': '.obj', "voxel_size" : 0.0020,  'number_of_points': 20000  , 'rotate':[
                                                                                                                        [[0, 0, pi/2],      2, 6], 
                                                                                                                        [[pi/2, 0, -pi/2],  2, 6],
                                                                                                                        # [[0, pi*3/4, 0], 12, -1]
                                                                                                                        ]},
    1:  {'name': 'a_bearing_holder'     , 'type': '.stl', "voxel_size" : 0.0007,  'number_of_points': 20000  , 'rotate':[
                                                                                                                        [[0, 0, 0],     1, 7], 
                                                                                                                        [[pi/2, 0, 0],  1, 8]
                                                                                                                        ]},
    # 2:  {'name': 'a_cam_zig'            , 'type': '.stl', "voxel_size" : 0.0015,  'number_of_points': 20000  , 'rotate':[
    2:  {'name': 'a_cam_zig'            , 'type': '.stl', "voxel_size" : 0.0020,  'number_of_points': 20000  , 'rotate':[
                                                                                                                        [[0, 0, 0],      2, 9], 
                                                                                                                        [[-pi/2, 0, 0],  2, 10], 
                                                                                                                        # [[pi*4/6, 0, 0], 2, -1] 
                                                                                                                        ]},
    3:  {'name': 'a_hex_wrench'         , 'type': '.obj', "voxel_size" : 0.0008, 'number_of_points': 20000  , 'rotate':[
                                                                                                                        [[0, 0, pi/2],  1, 12],
                                                                                                                        [[pi, 0, -pi/2], 1, 11]
                                                                                                                        ]},
    4:  {'name': 'a_rod_end_bearing'    , 'type': '.stl', "voxel_size" : 0.0015, 'number_of_points': 20000  , 'rotate':[
                                                                                                                        # [[0, 0, 0],    1, 14], 
                                                                                                                        [[0, pi/2, pi/2], 2, 14], 
                                                                                                                        [[pi/2, 0, pi], 2, 13]    
                                                                                                                        ]},
    # 5:  {'name': 'a_roller'             , 'type': '.stl', "voxel_size" : 0.0005,  'number_of_points': 20000  , 'rotate':[
    5:  {'name': 'a_roller'             , 'type': '.obj', "voxel_size" : 0.0007,  'number_of_points': 20000  , 'rotate':[
                                                                                                                        [[0, 0, 0],     1, 15], 
                                                                                                                        [[pi/2, 0, pi/23],  1, 16]  
                                                                                                                        ]},

    # 6:  {'name': 'g_L_shape_angle'      , 'type': '.obj', 'voxel_size': 0.001   , 'number_of_points': 5000  , 'rotate': 13 },
    # 7:  {'name': 'g_bearing_holder_x'   , 'type': '.obj', 'voxel_size': 0.0005  , 'number_of_points': 5000  , 'rotate': 1  },
    # 8:  {'name': 'g_bearing_holder_y'   , 'typze': '.obj', 'voxel_size': 0.001   , 'number_of_points': 5000  , 'rotate': 1  },
    # 9:  {'name': 'g_cam_zig_x'          , 'type': '.obj', 'voxel_size': 0.0007  , 'number_of_points': 5000  , 'rotate': 1  },
    # 10: {'name': 'g_cam_zig_y'          , 'type': '.obj', 'voxel_size': 0.002   , 'number_of_points': 5000  , 'rotate': 2  },
    # 11: {'name': 'g_hex_wrench_x'       , 'type': '.obj', 'voxel_size': 0.0025  , 'number_of_points': 5000  , 'rotate': 2  },
    # 12: {'name': 'g_hex_wrench_y'       , 'type': '.stl', 'voxel_size': 0.002   , 'number_of_points': 10000 , 'rotate': 13 },
    # 13: {'name': 'g_rod_end_bearing_x'  , 'type': '.obj', 'voxel_size': 0.00075 , 'number_of_points': 5000  , 'rotate': 2  },
    # 14: {'name': 'g_rod_end_bearing_y'  , 'type': '.obj', 'voxel_size': 0.0008  , 'number_of_points': 5000  , 'rotate': 2  },
    # 15: {'name': 'g_roller_x'           , 'type': '.obj', 'voxel_size': 0.0005  , 'number_of_points': 5000  , 'rotate': 12 },
    # 16: {'name': 'g_roller_y'           , 'type': '.obj', 'voxel_size': 0.0005  , 'number_of_points': 5000  , 'rotate': 1  },

    # 6:  {'name': 'g_L_shape_angle'      , 'type': '.obj', 'voxel_size': 0.001   , 'number_of_points': 5000  , 'rotate': 12 },
    # 7:  {'name': 'g_bearing_holder_x'   , 'type': '.obj', 'voxel_size': 0.0007  , 'number_of_points': 5000  , 'rotate': 1  },
    # 8:  {'name': 'g_bearing_holder_y'   , 'type': '.obj', 'voxel_size': 0.0015  , 'number_of_points': 5000  , 'rotate': 1  },
    # 9:  {'name': 'g_cam_zig_x'          , 'type': '.obj', 'voxel_size': 0.001   , 'number_of_points': 5000  , 'rotate': 1  },
    # 10: {'name': 'g_cam_zig_y'          , 'type': '.obj', 'voxel_size': 0.002   , 'number_of_points': 5000  , 'rotate': 2  },
    # 11: {'name': 'g_hex_wrench_x'       , 'type': '.obj', 'voxel_size': 0.0025  , 'number_of_points': 5000  , 'rotate': 2  },
    # # 12: {'name': 'g_hex_wrench_y'       , 'type': '.stl', 'voxel_size': 0.0015  , 'number_of_points': 20000 , 'rotate': 12 },
    # 12: {'name': 'g_hex_wrench_y'       , 'type': '.stl', 'voxel_size': 0.0015  , 'number_of_points': 20000 , 'rotate': 1 },
    # 13: {'name': 'g_rod_end_bearing_x'  , 'type': '.obj', 'voxel_size': 0.001   , 'number_of_points': 5000  , 'rotate': 2  },
    # 14: {'name': 'g_rod_end_bearing_y'  , 'type': '.obj', 'voxel_size': 0.001   , 'number_of_points': 5000  , 'rotate': 2  },
    # 15: {'name': 'g_roller_x'           , 'type': '.obj', 'voxel_size': 0.0008  , 'number_of_points': 5000  , 'rotate': 12 },
    # 16: {'name': 'g_roller_y'           , 'type': '.obj', 'voxel_size': 0.0008  , 'number_of_points': 5000  , 'rotate': 1  },

    6:  {'name': 'g_L_shape_angle'      , 'type': '.obj', 'voxel_size': 0.0008  , 'number_of_points': 30000  , 'rotate': 1  },
    7:  {'name': 'g_bearing_holder_x'   , 'type': '.obj', 'voxel_size': 0.0007  , 'number_of_points': 30000  , 'rotate': 1  },
    8:  {'name': 'g_bearing_holder_y'   , 'type': '.obj', 'voxel_size': 0.0015  , 'number_of_points': 30000  , 'rotate': 1  },
    9:  {'name': 'g_cam_zig_x'          , 'type': '.obj', 'voxel_size': 0.0010  , 'number_of_points': 30000  , 'rotate': 1  },
    10: {'name': 'g_cam_zig_y'          , 'type': '.obj', 'voxel_size': 0.0020  , 'number_of_points': 30000  , 'rotate': 2  },
    11: {'name': 'g_hex_wrench_x'       , 'type': '.obj', 'voxel_size': 0.0025  , 'number_of_points': 30000  , 'rotate': 2  },
    12: {'name': 'g_hex_wrench_y'       , 'type': '.stl', 'voxel_size': 0.0015  , 'number_of_points': 30000  , 'rotate': 1 },
    13: {'name': 'g_rod_end_bearing_x'  , 'type': '.obj', 'voxel_size': 0.0010  , 'number_of_points': 30000  , 'rotate': 2  },
    14: {'name': 'g_rod_end_bearing_y'  , 'type': '.obj', 'voxel_size': 0.0010  , 'number_of_points': 30000  , 'rotate': 2  },
    15: {'name': 'g_roller_x'           , 'type': '.obj', 'voxel_size': 0.0008  , 'number_of_points': 30000  , 'rotate': 1 },
    16: {'name': 'g_roller_y'           , 'type': '.obj', 'voxel_size': 0.0008  , 'number_of_points': 30000  , 'rotate': 1  },

    # 6:  {'name': 'g_L_shape_angle'      , 'type': '.obj', 'voxel_size': 0.002   , 'number_of_points': 5000  , 'rotate': 13 },
    # 7:  {'name': 'g_bearing_holder_x'   , 'type': '.obj', 'voxel_size': 0.001  , 'number_of_points': 5000  , 'rotate': 1  },
    # 8:  {'name': 'g_bearing_holder_y'   , 'type': '.obj', 'voxel_size': 0.002   , 'number_of_points': 5000  , 'rotate': 1  },
    # 9:  {'name': 'g_cam_zig_x'          , 'type': '.obj', 'voxel_size': 0.001  , 'number_of_points': 5000  , 'rotate': 1  },
    # 10: {'name': 'g_cam_zig_y'          , 'type': '.obj', 'voxel_size': 0.003   , 'number_of_points': 5000  , 'rotate': 2  },
    # 11: {'name': 'g_hex_wrench_x'       , 'type': '.obj', 'voxel_size': 0.003  , 'number_of_points': 5000  , 'rotate': 2  },
    # 12: {'name': 'g_hex_wrench_y'       , 'type': '.stl', 'voxel_size': 0.003   , 'number_of_points': 10000 , 'rotate': 13 },
    # 13: {'name': 'g_rod_end_bearing_x'  , 'type': '.obj', 'voxel_size': 0.001 , 'number_of_points': 5000  , 'rotate': 2  },
    # 14: {'name': 'g_rod_end_bearing_y'  , 'type': '.obj', 'voxel_size': 0.001  , 'number_of_points': 5000  , 'rotate': 2  },
    # 15: {'name': 'g_roller_x'           , 'type': '.obj', 'voxel_size': 0.001  , 'number_of_points': 5000  , 'rotate': 12 },
    # 16: {'name': 'g_roller_y'           , 'type': '.obj', 'voxel_size': 0.001  , 'number_of_points': 5000  , 'rotate': 1  },
}
