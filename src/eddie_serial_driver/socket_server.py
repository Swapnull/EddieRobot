from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys


class Eddie_Server:

    port_number = 5000
    size = 1024

    host_name = None
    my_socket = None

    def __init__(self):
        self.host_name = gethostbyname('0.0.0.0')
        self.my_socket = socket( AF_INET, SOCK_DGRAM )
        self.my_socket.bind((self.host_name, self.port_number))
        print ("Test server listening on port {0}\n".format(self.port_number))

    def listen_for_port_data(self):
        (data, addr) = self.my_socket.recvfrom(self.size)
        return data

    def close_port(self):
        self.my_socket.close()
        sys.ext()
