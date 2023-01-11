import socket
from people_tracker import PeopleTracker
from drone import Drone


def calculate_flight_path(topleft, botright, scan_radius_meters=20):
    # Latitude | Longitude
    # y        | x

    # Convert meters to GPS degree
    scan_radius = scan_radius_meters / 111000

    gps_sequence = [[botright[0] + scan_radius, topleft[1] + scan_radius],  # Bottom left
                    [topleft[0] - scan_radius, topleft[1] + scan_radius]]   # Top left
    gps_last_location = gps_sequence[-1]
    while gps_last_location[1] + scan_radius < botright[1]:
        # Move one scan_radius to the right
        gps_sequence.append([gps_last_location[0], gps_last_location[1] + scan_radius])
        gps_last_location = gps_sequence[-1]
        # Case it is in the top
        if gps_last_location[0] > botright[0] + scan_radius:
            gps_sequence.append([botright[0] + scan_radius, gps_last_location[1]])
        # Case it is bottom
        else:
            gps_sequence.append([topleft[0] - scan_radius, gps_last_location[1]])
        gps_last_location = gps_sequence[-1]
    return gps_sequence


def main():

    # Setup people tracking
    tracker = PeopleTracker()

    # Setup networking, connection to drone and frontend
    # create a socket object
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # get local machine name
    host = socket.gethostname()

    port = 8888

    # bind to the port
    server.bind((host, port))

    # queue up to 3 request
    server.listen(3)

    print(str(socket.gethostbyname(host)) + ':' + str(port))

    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = server.accept()
    print(f'connection from {client_address}')

    # Recieve area to search and calculate flight path
    # Send flight path to drone

    found_people = {}
    input()
    drone = Drone()

    while True:
        # Get bounding boxes from drone and GPS locations from drone

        gps, bboxes = drone.update()

        new_people = tracker.update(bboxes)

        found_people_ids = found_people.keys()
        for person_id in new_people.keys():
            if person_id not in found_people_ids:
                found_people[person_id] = gps
                # send new person id and GPS
                message = str(person_id) + '_' + str(gps[0]) + '_' + str(gps[1]) + '\n'
                print('sending message: ' + message)
                connection.sendall(message.encode(encoding='UTF-8'))


if __name__ == '__main__':
    main()
