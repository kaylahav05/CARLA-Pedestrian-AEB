import carla;

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    print("Successfully connected to carla")
    print("Current map: ", world.get_map().name)

if __name__ == "__main__":
    main()