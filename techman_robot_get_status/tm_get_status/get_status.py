from tm_get_status import translate_jason_to_list

import rclpy

from rclpy.node import Node
from threading import Thread
import socket
import time

class Talker(Node):

    def __init__(self,ip):
        super().__init__('talker')
        self.ip = ip
        self.prot = 5891
        self.socketConnect =None
        self.isConnect = False

    def socket_connect(self):
        self.socketConnect = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socketConnect.connect((self.ip, self.prot))
        self.isConnect = True
        self.backgroundListen = Thread(target = self.listener_callback)
        self.backgroundListen.start()

    def listener_callback(self):
        remainString =''
        while self.isConnect:
            dataByte = self.socketConnect.recv(1024)
            print("get data from server which is")
            data=str(dataByte, encoding = "utf-8")
            print(data)
            data= data+remainString
            print("new data is")
            remainString,newString = translate_jason_to_list.TmJasonToDiction.split_package(data)
            if(newString is not None):
                print(newString[0])
                jasonString = translate_jason_to_list.TmJasonToDiction.tm_string_to_jason(newString[0])
                print(jasonString)
                dictionary = translate_jason_to_list.TmJasonToDiction.jason_to_dic(jasonString)
                print(dictionary)

            
            #time.sleep(0.1)  
            #msg = PythonTalker()
            #msg.data = "Hello World!"
            #msg.talk_times = self.counter
            #self.counter += 1
            #self.get_logger().info('Publishing something !')
            #self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ip = "192.168.132.242"

    node = Talker(ip)

    node.socket_connect()

    node.listener_callback()

    rclpy.spin(node)

    node.destroy_node()
    Talker.isConnect = False
    rclpy.shutdown()


if __name__ == '__main__':
    main()