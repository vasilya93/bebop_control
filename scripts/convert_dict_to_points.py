from geometry_msgs.msg import Point

def convert_dict_to_points(dict_markers):
    dict_marker_points = {}
    for key in dict_markers:
        point = Point()
        point.x = eval(dict_markers[key]["x"])
        point.y = eval(dict_markers[key]["y"])
        point.z = eval(dict_markers[key]["z"])
        dict_marker_points[key] = point
    return dict_marker_points

