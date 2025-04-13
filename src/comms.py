import socket
import pickle
import struct

#Function to create a TCP Server to send data to. 
def server_TCP(ip ='127.0.0.1',port =65432):

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((ip, port))
    return client
    

def create_packet(R,G,B, X, Y):
    return struct.pack('>iiidd',R,G,B,X,Y)


def send_tcp_data(client_socket, packets):
    for byte_data in packets:
        client_socket.sendall(packet)

def recieve_tcp_data(server, buffer_size = 1024):
    while True: 
        conn,addr = server.accept() 

        data = conn.recv(buffer_size)
        if not data: 
            break 

    server.close()
    return data