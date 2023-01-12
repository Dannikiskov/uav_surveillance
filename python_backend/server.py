import socket, ast
from people_tracker import PeopleTracker


def calculate_flight_path(topleft, botright, scan_radius_meters=20):
    # Latitude | Longitude
    # y        | x

    # Convert meters to GPS degrees
    scan_radius_degrees = scan_radius_meters / 111000

    gps_sequence = [[botright[0] + scan_radius_degrees, topleft[1] + scan_radius_degrees],  # Bottom left
                    [topleft[0] - scan_radius_degrees, topleft[1] + scan_radius_degrees]]   # Top left
    gps_last_location = gps_sequence[-1]
    while gps_last_location[1] + scan_radius_degrees < botright[1]:
        # Move one scan_radius to the right
        gps_sequence.append([gps_last_location[0], gps_last_location[1] + scan_radius_degrees])
        gps_last_location = gps_sequence[-1]
        # Case it is in the top
        if gps_last_location[0] > botright[0] + scan_radius_degrees:
            gps_sequence.append([botright[0] + scan_radius_degrees, gps_last_location[1]])
        # Case it is bottom
        else:
            gps_sequence.append([topleft[0] - scan_radius_degrees, gps_last_location[1]])
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

    print('Backend server IP:', str(socket.gethostbyname(host)) + ':' + str(port))

    # bind to the port
    server.bind((host, port))

    print(str(socket.gethostbyname(host)) + ':' + str(port))

    # allow maximum 1 connection to
    # the socket
    server.listen(1)
    
    # wait till a client accept
    # connection
    client, addr = server.accept()

    # display client address
    print('CONNECTION FROM:', str(addr))

    # Recieve area to search and calculate flight path
    # Send flight path to drone
    flight_path = calculate_flight_path([55.41660012905264, 10.371145382523537], [55.41559112178513, 10.37313625216484])
    print('Ready to send flight path:', flight_path)
    input()
    flight_path_msg = 'route=' + str(flight_path)
    client.send(flight_path_msg.encode())

    found_people = {}

    while True:
        # Get bounding boxes from drone and GPS locations from drone
        msg = client.recv(1024)
        if msg:
            decoded_msg = msg.decode()
            if 'reached' in decoded_msg:
                print(decoded_msg)
            else:
                gps, bboxes = ast.literal_eval(msg.decode())
                print('gps:', gps)
                print('bboxes:', bboxes)
    
                new_people = tracker.update(bboxes)
        
                found_people_ids = found_people.keys()
                for person_id in new_people.keys():
                    if person_id not in found_people_ids:
                        found_people[person_id] = gps
                        # send new person id and GPS
                        message = str(person_id) + '_' + str(gps[0]) + '_' + str(gps[1]) + '\n'
                        print('sending message: ' + message)
                        #connection.sendall(message.encode(encoding='UTF-8'))


if __name__ == '__main__':
    main()


