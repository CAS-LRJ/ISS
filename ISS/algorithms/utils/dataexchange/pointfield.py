class ISSPointField(object):

    def __init__(self) -> None:
        pass

    def read_from_ros_msg(self, point_field_msg):
        self.name = point_field_msg.name
        self.offset = point_field_msg.offset
        self.datatype = point_field_msg.datatype
        self.count = point_field_msg.count 