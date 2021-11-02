# from ..interpreter import model


# Commented out to make static node recovery be used
# @model('map_server', 'map_server')
def map_server(c):
    c.read('~frame_id', 'map')
    c.read('~negate', 0)
    c.read('~occupied_thresh', 0.65)
    c.read('~free_thresh', 0.196)

    c.provide('static_map', 'nav_msgs/GetMap')

    c.pub('map_metadata', 'nav_msgs/MapMetaData')
    c.pub('map', 'nav_msgs/OccupancyGrid')
