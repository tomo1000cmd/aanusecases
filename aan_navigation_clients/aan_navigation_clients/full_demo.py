import subprocess

def main(args=None):
    try:
        print("Starting field coverage")
        node = subprocess.Popen(["ros2", "run", "aan_navigation_clients", "field_cover_client"], text=True)
        node.wait()

        print("Starting row following")
        node = subprocess.Popen(["ros2", "run", "aan_navigation_clients", "row_follow_client"], text=True)
        node.wait()

        print("Starting docking client")
        node = subprocess.Popen(["ros2", "run", "aan_navigation_clients", "docking_client"], text=True)
        node.wait()

    except KeyboardInterrupt:
        print("Stopping all nodes")
        node.kill()


if __name__ == '__main__':
    main()
