import os
import sys
import getopt
import socket


if __name__ == "__main__":
    host = '0.0.0.0'
    port = 8000
    addr = (host, port)
    udpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpServer.bind(addr)
    while True:
        data, addr = udpServer.recvfrom(1024)
        msg = str(data, encoding = 'utf-8')
        print(msg)
        if msg == "YAHBOOMARM_FIND":
            udpServer.sendto(bytes("$I am jetson version!\n", encoding='utf-8'), addr)
            print("send ok")