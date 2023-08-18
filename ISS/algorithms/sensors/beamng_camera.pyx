from beamngpy.sensors import Camera as CameraBng

class Camera(object):

    def __init__(self):
        self.data_source = None
        self.camera_obj = None
        self.depth_image = None
        self.render_depth = None
        self.rgb_image = None   
        self.render_rgb = None                     
    
    def from_beamng(self, 
                bng, 
                veh, 
                requested_update_time=0.01, 
                update_priority=1.0, 
                is_using_shared_memory=True, 
                pos=(0, -2, 1),
                dir=(0, -1, 0),
                up=(0, 0, 1),
                field_of_view_y=70.,
                near_far_planes=(0.1, 100),
                resolution=(640, 456),
                is_render_colour = True,
                is_render_annotations=False,
                is_render_instance=False,
                is_render_depth=True,):
        self.camera_obj = CameraBng('camera', bng, veh, requested_update_time, update_priority, pos, dir, up, resolution, field_of_view_y, near_far_planes, is_using_shared_memory, is_render_colour, is_render_annotations, is_render_instance, is_render_depth)
        self.data_source = 'BeamNG'
        self.render_depth = is_render_depth
        self.render_rgb = is_render_colour

    def poll_data(self):
        if self.data_source == 'BeamNG':
            images = self.camera_obj.poll()
            if self.render_rgb:
                self.rgb_image = images['colour'].convert('RGB')
            if self.render_depth:
                self.depth_image = images['depth'].convert('RGB')        