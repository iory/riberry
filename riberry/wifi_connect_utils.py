import socket


def send_wifi_control_command(command, socket_path='\0wifi_connect'):
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(socket_path)
    client.send(command.encode())
    client.close()
