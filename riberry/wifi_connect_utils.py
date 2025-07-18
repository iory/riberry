import socket


def send_wifi_connect_command(command, socket_path='\0wifi_connect'):
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(socket_path)
    client.send(command.encode())
    client.close()


def get_wifi_connect_status(socket_path='\0wifi_connect', timeout=0.1):
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout)

    try:
        client.connect(socket_path)
        client.send(b"status")
        response = client.recv(1024).decode().strip()
        print(f"wifi-connect status: {response}")
        return response
    except socket.timeout:
        print(f"Get Wifi Connect Status Error: Operation timed out after {timeout} seconds.")
        return None
    except Exception as e:
        print(f"Error communicating with server: {e}")
    finally:
        client.close()
