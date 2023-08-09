import sys
import rclpy
import queue
import signal
from rclpy.node import Node

from sensor_msgs.msg import Image 

from flask import Flask, request, jsonify
import numpy as np
import cv2
from waitress import serve
from datetime import datetime
from cv_bridge import CvBridge
import threading


class ImagePub(Node):
    def __init__(self, node_name, is_test, path):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Image, 'techman_image', 10)
        self.con = threading.Condition()
        self.image_q = queue.Queue()
        self.leave_thread = False
        if is_test:
            self.thread = threading.Thread(target=self.pub_data_thread, args=(False,))
            timer_period = 1.0
            self.img = cv2.imread(path)
            self.tmr = self.create_timer(timer_period, self.publish_test_image)
        else:
            self.thread = threading.Thread(target=self.pub_data_thread, args=(True,))
        self.thread.start()
                          
    def set_image_and_notify_send(self, img):
        self.con.acquire()
        self.image_q.put(img)
        self.con.notify()
        self.con.release()

    def signal_handler(self, _, __):
        self.close_thread()
        sys.exit(0)
        
    def publish_test_image(self):
        self.img = cv2.flip(self.img, 1)
        self.set_image_and_notify_send(self.img)

    def image_publisher(self, image):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(image)
        self.get_logger().info('Publishing something!, queue size is ' + str(self.image_q.qsize()))
        self.publisher.publish(msg)
    
    def close_thread(self):
        self.leave_thread = True
        self.con.acquire()
        self.con.notify()
        self.con.release()
        
    def pub_data_thread(self, is_request_data):
        self.con.acquire()
        while rclpy.ok():
            self.con.wait()
            while not self.image_q.empty():
                if is_request_data:
                    file2np = np.fromstring(self.image_q.get(), np.uint8)
                    img = cv2.imdecode(file2np, cv2.IMREAD_UNCHANGED)
                    self.image_publisher(img)
                else:
                    self.image_publisher(self.image_q.get())
            if self.leave_thread:
                break
        self.con.release()

    @staticmethod
    def fake_result(m_method):
        # classification
        if m_method == 'CLS':
            # inference img here
            result = {
                "message": "success",
                "result": "NG", 
                "score": 0.987
            }
        # detection
        elif m_method == 'DET':            
            # inference img here                                    
            result = {
                "message": "success",
                "annotations": 
                [
                    { 
                        "box_cx": 150,
                        "box_cy": 150,
                        "box_w": 100,
                        "box_h": 100,                    
                        "label": "apple",                    
                        "score": 0.964,
                        "rotate": -45
                    },
                    { 
                        "box_cx": 550,
                        "box_cy": 550,
                        "box_w": 100,
                        "box_h": 100,
                        "label": "car",
                        "score": 1.000,
                        "rotation": 0
                    },
                    { 
                        "box_cx": 350,
                        "box_cy": 350,
                        "box_w": 150,
                        "box_h": 150,
                        "label": "mobilephone",
                        "score": 0.886,
                        "rotation": 135
                    }
                ],
                "result": None
            }
        # no method
        else:
            result = {            
                "message": "no method",
                "result": None            
            }
        return result

    @staticmethod
    def get_none():
        print('\n[{0}] [{1}] -> Get()'.format(request.environ['REMOTE_ADDR'], datetime.now()))
        # user defined method
        result = {
            "result": "api",
            "message": "running",
        } 
        return jsonify(result)

    @staticmethod
    def get(m_method):
        print('\n[{0}] [{1}] -> Get({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))
        # user defined method
        if m_method == 'status':
            result = {
                "result": "status",
                "message": "im ok"
            }
        else:
            result = {
                "result": "fail",
                "message": "wrong request"            
            }
        return jsonify(result)

    def post(self, m_method):
        print('\n[{0}] [{1}] -> Post({2})'.format(request.environ['REMOTE_ADDR'], datetime.now(), m_method))          
        # get key/value
        model_id = request.args.get('model_id')
        print('model_id: {}'.format(model_id))

        # check key/value
        if model_id is None:
            print("model_id is not set")    
            result = {
                "message": "fail",
                "result": "model_id required"
            }  
            return jsonify(result)

        # convert image data        
        # file2np = np.fromstring(request.files['file'].read(), np.uint8)
        # img = cv2.imdecode(file2np, cv2.IMREAD_UNCHANGED)
        # cv2.imwrite('test.png',img)

        self.set_image_and_notify_send(request.files['file'].read())

        result = self.fake_result(m_method)    

        return jsonify(result)


def set_route(app, node):
    app.route('/api/<string:m_method>', methods=['POST'])(node.post)
    app.route('/api/<string:m_method>', methods=['GET'])(node.get)
    app.route('/api', methods=['GET'])(node.get_none)


def main():
    rclpy.init(args=None)
    test = False
    app = Flask(__name__)
    if test:
        try:
            print(sys.argv[1:])
        except IndexError:
            print("arg is not correct!")
            return 1
        
        node = ImagePub('image_pub', test, sys.argv[1])
    else:
        node = ImagePub('image_pub', test, None)

        set_route(app, node)
        print("Listening on an ip port:6189 combination")
        serve(app, port=6189)
    signal.signal(signal.SIGINT, node.signal_handler)
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
