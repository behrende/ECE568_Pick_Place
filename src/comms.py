import socket
import pickle
import struct

#Function to create a TCP Server to send data to. 
def server_TCP(ip ='127.0.0.1',port =65432):

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((ip, port))
    return client



def receive_tcp_data(server, buffer_size=1024):
    """
    Accepts a connection on the given server socket and prints incoming data.
    """
    try:
        while True:
            conn, addr = server.accept()
            print(f"[+] Connection from {addr}")

            with conn:
                while True:
                    data = conn.recv(buffer_size)
                    if not data:
                        break
                    print(f"[*] Received from {addr}: {data}")
    except KeyboardInterrupt:
        print("\n[!] Server shutting down")
    finally:
        server.close()


    

def create_packet(R,G,B, X, Y):
    #print(type(r))
   # print(type(g))
  #  print(type(b))
 #   print(type(x))
#    print(type(y))

    return struct.pack('>iiidd',R,G,B,X,Y)


def send_tcp_data(client_socket, data):
    for byte_data in data:
        packet = create_packet(byte_data["color"], byte_data["X"], byte_data["Y"])
        client_socket.sendall(packet)
