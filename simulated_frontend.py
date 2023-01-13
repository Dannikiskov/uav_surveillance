import socket


def main():

    host = 'localhost'
    port = 8888

    # Create a client socket
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect it to the server
    print('Connecting to server')
    connected = False
    while not connected:
        try:
            client.connect((host, port))
            connected = True
            client.send('frontend'.encode())
        except socket.error as e:
            pass

    area_to_search_msg = '[55.3518, 10.4052], [55.3513, 10.4061]'
    print('GPS coordinates for area to search: ' + area_to_search_msg)
    input('Press enter to send GPS coordinates')
    client.send(area_to_search_msg.encode())

    while connected:
        try:
            msg = str(client.recv(1024).decode())
            msg_split = msg.split('_')
            print('New person found, ID:' + msg_split[0] + ' GPS:[' + msg_split[1] + ', ' + msg_split[2].rstrip() + ']')
        except socket.error as e:
            connected = False

    print('Connection lost, closing down')
    client.close()


if __name__ == '__main__':
    main()

