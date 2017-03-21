# Single spherical depth image from a 3D laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (clockwise, if Z is down) and
                         # around the positive Y axis (counterclockwise, if Y is on the left)
                         # with zero angle being forward along the x axis
             
float32 horizontal_fov   # horizontal field of view [deg]            
float32 vertical_fov     # vertical field of view [deg]   

sensor_msgs/Image image  # spherical depth image [uint16]
