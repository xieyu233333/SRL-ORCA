import struct
import time
import numpy as np
import socket,threading

def tcpserver(ri):
    # 创建socket
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 本地信息
    address = ('192.168.1.27', 5001)

    # 绑定
    tcp_server_socket.bind(address)

    tcp_server_socket.listen(128)

    while True:
        # 等待新的客户端连接
        client_socket, clientAddr = tcp_server_socket.accept()
        while True:
            # time.sleep(0.1)
            # 接收对方发送过来的数据
            recv_data = client_socket.recv(1024)  # 接收1024个字节
            print('recving')
            if recv_data:
                data=recv_data.decode('gbk')
                split_data=data.split(',')
                if  split_data[0].replace('.','').isdigit():
                    if split_data[1].replace('.','').isdigit():
                        x=float(split_data[0]) * 12
                        y=float(split_data[1]) * 12 + 1.93977
                        ri[0][0] = x
                        ri[0][1] = y
                        # print('接收到的数据为:x=', x , "y=" , y )
                        print('接收到的数据为:r=', ri)
                # print('接收到的数据为:', recv_data.decode('gbk'))
            else:
                break

        client_socket.close()

    tcp_server_socket.close()
def main():
    # 创建ri变量
    ri = np.zeros((1, 2))

    # 创建TCP服务器线程并传入ri变量
    server_thread = threading.Thread(target=tcpserver, args=(ri,))
    server_thread.start()

    # 主线程继续运行其他任务
    # while True:
    #     # 运行其他任务
    #     print("ri=",ri)

if __name__ == '__main__':
    main()