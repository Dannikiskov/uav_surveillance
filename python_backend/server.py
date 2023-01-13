import socket, ast, math
from people_tracker import PeopleTracker


# Calculates a flight path for the drone, given the top left and bottom right corners
# of the area to search.
# Returns a list of waypoints that is ordered in the sequence of the flight route
def calculate_flight_path(topleft, botright, scan_radius_meters=10):

    # Helper function, returns a new latitude that is increased by the scan_radius_meters amount
    def increase_lat(lat):
        earth_radius = 6378.137
        m_to_lat = (1 / ((2 * math.pi / 360) * earth_radius)) / 1000
        return lat + (scan_radius_meters * m_to_lat)

    # Helper function, returns a new longitude that is increased by the scan_radius_meters amount
    def increase_lon(lat_lon_arr):
        earth_radius = 6378.137
        m_to_lon = (1 / ((2 * math.pi / 360) * earth_radius)) / 1000
        return lat_lon_arr[1] + (scan_radius_meters * m_to_lon) / math.cos(lat_lon_arr[0] * (math.pi / 180))

    # The flight path to be returned, which begins in bottom left of the area and proceeds to the top left
    gps_sequence = [[increase_lat(botright[0]), increase_lon(topleft)],  # Bottom left
                    [topleft[0] - (increase_lat(topleft[0]) - topleft[0]), increase_lon(topleft)]]   # Top left

    # To store which waypoint it has reached so fair in its flight path calculation
    gps_last_location = gps_sequence[-1]

    # Keeps going until it reaches the end of the area
    while increase_lon(gps_last_location) < botright[1]:
        # Move one scan_radius to the right
        gps_sequence.append([gps_last_location[0], increase_lon(gps_last_location)])
        gps_last_location = gps_sequence[-1]
        # Case it is in the top
        if gps_last_location[0] > increase_lat(botright[0]):
            gps_sequence.append([increase_lat(botright[0]), gps_last_location[1]])
        # Case it is bottom
        else:
            gps_sequence.append([topleft[0] - (increase_lat(topleft[0]) - topleft[0]), gps_last_location[1]])
        gps_last_location = gps_sequence[-1]

    return gps_sequence


def main():

    # Setup people tracking
    tracker = PeopleTracker()

    # Setup networking, connection to drone and frontend
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Setup server IP
    host = 'localhost'
    port = 8888

    print('Server IP:', host + ':' + str(port))

    # Bind the server to the port
    server.bind((host, port))

    # Allow a maximum of two connections to the server for the drone and frontend
    server.listen(2)

    # Dictionary for the connected clients, which will be 'drone' and 'frontend'
    clients = {}

    # Wait for connection from the drone and the frontend
    for _ in range(2):
        client, addr = server.accept()
        client_name = client.recv(1024).decode()
        clients[client_name] = client
        # Display client address
        print('Connection from ' + client_name + ': ' + str(addr))

    # Recieve area to search and calculate flight path
    topleft, botright = ast.literal_eval(clients['frontend'].recv(1024).decode())
    print(topleft, botright)
    print('Received coordinates, calculating flight path')
    flight_path = calculate_flight_path(topleft, botright)

    # Send flight path to drone
    print('Sending flight path to drone')
    message = 'route=' + str(flight_path)
    clients['drone'].send(message.encode())

    found_people = {}

    connected = True

    # Main server logic loop, that keeps going until it is disconnected
    while connected:
        try:
            # Get bounding boxes and GPS location from drone
            msg = clients['drone'].recv(1024)
            if msg:
                decoded_msg = str(msg.decode())
                # Case it is a connection check from the drone, then do nothing
                if 'Connection check' in decoded_msg:
                    pass
                # Case it is a message containing boundingboxes and the GPS location
                else:
                    gps, bboxes = ast.literal_eval(decoded_msg)

                    # Gets a dictionary of the found people ID's as keys
                    # and their GPS locations as values
                    new_people = tracker.update(bboxes)

                    found_people_ids = found_people.keys()
                    for person_id in new_people.keys():
                        # Case the person has not already been spotted
                        if person_id not in found_people_ids:
                            # Add the new person
                            found_people[person_id] = gps

                            # Sends the new person's ID and GPS to the server
                            message = str(person_id) + '_' + str(gps[0]) + '_' + str(gps[1]) + '\n'
                            print('Found new person, sending to frontend ID:' + str(person_id) + ' GPS:[' + str(gps[0]) + ', ' + str(gps[1]) + ']')
                            clients['frontend'].send(message.encode(encoding='UTF-8'))
        except socket.error as e:
            connected = False
            print('Connection lost')


if __name__ == '__main__':
    main()


