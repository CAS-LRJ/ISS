import carla

carla_host = "127.0.0.1"
carla_port = 2000

client = carla.Client(carla_host, carla_port)
client.set_timeout(5.0)

world = client.load_world("Town06")


